#ifndef SCENE_MANIPULATION_SERVICE_H
#define SCENE_MANIPULATION_SERVICE_H

#include <unordered_map>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/timer.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include "ros2_scene_manipulation_msgs/srv/manipulate_scene.hpp"
#include "related_transform.h"

class SceneManipulationService : public rclcpp::Node
{
public:
	using ManipulateScene = ros2_scene_manipulation_msgs::srv::ManipulateScene;
	using OnSceneObjectMoved = std::function<void(const RelatedTransform&)>;

    SceneManipulationService(
		const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
		OnSceneObjectMoved on_scene_object_moved = [](const RelatedTransform&){});

private:
    bool onSceneManipulationRequest(
		const std::shared_ptr<rmw_request_id_t> request_header,
  		const std::shared_ptr<ManipulateScene::Request> request,
  		const std::shared_ptr<ManipulateScene::Response> response);

	void broadcastActivetransforms();

	void broadcastTransform(const RelatedTransform& transform);

	bool updateRelatedTransform(RelatedTransform& related_transform);

    std::unordered_map<std::string, RelatedTransform>		active_transforms_;
	std::unordered_map<std::string, RelatedTransform>	    static_transforms_;
	rclcpp::Service<ManipulateScene>::SharedPtr				scene_manipulation_service_;
	rclcpp::TimerBase::SharedPtr							timer_;
	tf2_ros::TransformBroadcaster							transform_broadcaster_;
	tf2_ros::Buffer											tf_buffer_;
  	tf2_ros::TransformListener								transform_listener_;
	tf2_ros::StaticTransformBroadcaster						static_broadcaster_;
	std::mutex 												manipulating_scene_mutex_;
	OnSceneObjectMoved										on_scene_object_moved_;
};

#endif
