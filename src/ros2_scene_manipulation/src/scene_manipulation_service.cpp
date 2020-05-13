#include <memory>
#include <chrono>
#include <unordered_map>
#include <inttypes.h>
#include <iostream>
#include <fstream>

#include "read_transforms.h"
#include "scene_manipulation_service.h"
#include "conversions.h"

namespace {
	template<typename InputIt, class UnaryFunction>
	constexpr UnaryFunction for_each_second(InputIt first, InputIt last, UnaryFunction f)
	{
		for(; first != last; ++first){
			f((*first).second);
		}
		return f;
	}

	using std::placeholders::_1;
	using std::placeholders::_2;
	using std::placeholders::_3;
}


std::string readFile(const char* path)
{
    std::ifstream t(path);
    std::string file_contents((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    t.close();
    return file_contents;
}

SceneManipulationService::SceneManipulationService(
	  const rclcpp::NodeOptions& node_options
	, OnSceneObjectMoved on_scene_object_moved)
	: Node("scene_manipulation_service_node", node_options)
	, scene_manipulation_service_(create_service<ManipulateScene>("scene_manipulation_service", std::bind(&SceneManipulationService::onSceneManipulationRequest, this, _1, _2, _3)))
	, timer_(this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&SceneManipulationService::broadcastActivetransforms, this)))
	, transform_broadcaster_(this)
	, tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
  	, transform_listener_(tf_buffer_)
	, static_broadcaster_(this)
	, on_scene_object_moved_(on_scene_object_moved)
{
	for_each_second(std::begin(static_transforms_), std::end(static_transforms_), [this](const RelatedTransform& transform){
		on_scene_object_moved_(transform);
		static_broadcaster_.sendTransform(toMsg(transform));
	});

    rclcpp::ParameterValue default_value("");
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    std::string active_fn = this->declare_parameter("active_transforms",
                                                    default_value, descriptor).get<std::string>();
    std::string static_fn = this->declare_parameter("static_transforms",
                                                    default_value, descriptor).get<std::string>();

    std::string active_t = readFile(active_fn.c_str());
    std::string static_t = readFile(static_fn.c_str());
    this->active_transforms_ = readTransforms(active_t);
    this->static_transforms_ = readTransforms(static_t);
}

bool SceneManipulationService::onSceneManipulationRequest(
		const std::shared_ptr<rmw_request_id_t> request_header,
  		const std::shared_ptr<ManipulateScene::Request> request,
  		const std::shared_ptr<ManipulateScene::Response> response)
{
	(void)request_header;

	std::lock_guard<std::mutex> lock(manipulating_scene_mutex_);
    auto transform_iter = active_transforms_.find(request->frame_id);
    // If the scene object does not exist, add it and return.
    if (transform_iter == active_transforms_.end()){
        //ROS_INFO_STREAM("Adding " << request->frame_id << " expressed in " << request->parent_id << " to active objects.");
		auto result = active_transforms_.emplace(request->frame_id, RelatedTransform({request->frame_id, request->parent_id, request->transform}));
		on_scene_object_moved_((*result.first).second);
		broadcastTransform((*result.first).second);
        return true;
    }

	auto& related_transform = transform_iter->second;
	related_transform.frame_id = request->frame_id;
	related_transform.parent_id = request->parent_id;

    // If same_position_in_world is true, recalculate the transformation expressed in the parent that would result in
    // The object not moving with respect to world.
    if (request->same_position_in_world) {
        if (!updateRelatedTransform(related_transform)){
            //ROS_INFO_STREAM("Did not find " << request->frame_id << " expressed in " << request->parent_id << ".");
            return false;
        }
    }
	else {
		related_transform.transform = request->transform;
	}
	on_scene_object_moved_(related_transform);
   	broadcastTransform(related_transform);
    //ROS_INFO_STREAM("Updated position of sceneobject " << request->frame_id << ". It is now expressed in " << request->parent_id << ".");

	response->result = true;
	return true;
}

void SceneManipulationService::broadcastTransform(const RelatedTransform& transform)
{
    transform_broadcaster_.sendTransform(toMsg(transform));
}

void SceneManipulationService::broadcastActivetransforms()
{
	std::lock_guard<std::mutex> lock(manipulating_scene_mutex_);
	for_each_second(std::begin(active_transforms_), std::end(active_transforms_), [this](const RelatedTransform& transform){
		broadcastTransform(transform);
	});
}

bool SceneManipulationService::updateRelatedTransform(RelatedTransform& related_transform)
{
	try {
        auto transform_stamped = tf_buffer_.lookupTransform(
			related_transform.parent_id,
			related_transform.frame_id,
			rclcpp::Time(0));
		related_transform.transform = transform_stamped.transform;
        return true;
    }
    catch (tf2::TransformException ex){
        //ROS_ERROR("%s", ex.what());
    }
    return false;
}
