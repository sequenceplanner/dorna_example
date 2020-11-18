use r2r;
use failure::Error;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use crossbeam::channel;
use std::thread;

use rand::Rng;

#[derive(PartialEq, Clone, Copy, Debug)]
struct Tick;

#[derive(PartialEq, Clone, Debug)]
struct API_State {
    pub cmd: serde_json::Value,
    pub state: serde_json::Value,
    pub nodes: Vec<String>,
}

impl API_State {
    fn new() -> Self {
        let cmd = serde_json::json!({
            "r1": {
                "ref_pos": "pre_take"
            },
            "r3": {
                "ref_pos": "pre_take"
            },
            "control_box": {
                "blue_light": false
            },
            "camera": {
                "do_scan": false
            },
            "gripper": {
                "close": false
            },
            "cubes": {
                "make_cubes": false,
                "remove_cubes": false
            },
        });

        let state = serde_json::json!({
            "r1": {
                "act_pos": ""
            },
            "r2": {
                "act_pos": ""
            },
            "control_box": {
                "blue_light_on": false
            },
            "camera": {
                "scanning": false,
                "done": false,
                "result": 0,
            },
            "gripper": {
                "closed": false,
                "part_sensor": false,
            },
            "cubes": {
                "at_input_output_pos": false,
            },
        });

        API_State {
            cmd,
            state,
            nodes: vec!()
        }
    }

    fn upd_state(&mut self, pointer: &str, value: serde_json::Value) {
        self.state.pointer_mut(pointer).map(|v| *v = value).expect(&format!{"Could not add to state with pointer {}", pointer});
    }
    fn get_value_from_cmd(&self, pointer: &str) -> Option<&serde_json::Value> {
        self.cmd.pointer(pointer)
    }
}

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "api_node", "")?;
    let state = Arc::new(Mutex::new(API_State::new()));

    send_the_state(&mut node, state.to_owned());
    send_the_cmd(&mut node, state.to_owned());
    listner(&mut node, state.to_owned());

    // Maybe add a ticker for the cmd?

    loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    }
}



/// Send the state
fn send_the_state(node: &mut r2r::Node, state: Arc<Mutex<API_State>>) {
    let state_publisher = node.create_publisher::<r2r::std_msgs::msg::String>("state").expect("State publisher creation failed");
    let rx_out = channel::tick(Duration::from_millis(300));
    thread::spawn(move || loop {
        match rx_out.recv() {
            Ok(_) => {
                let x = state.lock().unwrap();
                let msg = r2r::std_msgs::msg::String { data: x.state.to_string() };
                state_publisher.publish(&msg).expect("Can not send the state!");
            }
            Err(e) => {
                panic!("Send state channel out did not work: {:?}", e);
            }
        }
    });
}

/// Send the commands
fn send_the_cmd(node: &mut r2r::Node, state: Arc<Mutex<API_State>>){
    let gripper_publisher = node.create_publisher_untyped("/gripper/goal", "gripper_msgs/msg/Goal").expect("Hmm, can not create node");
    let control_box_publisher = node.create_publisher_untyped("/control_box/goal", "control_box_msgs/msg/Goal").expect("Hmm, can not create node");
    let camera_publisher = node.create_publisher_untyped("/camera/goal", "camera_msgs/msg/Goal").expect("Hmm, can not create node");
    let r1_publisher = node.create_publisher_untyped("/dorna/r1/goal", "robot_msgs/msg/RobotGoal").expect("Hmm, can not create node");
    let r2_publisher = node.create_publisher_untyped("/dorna/r2/goal", "robot_msgs/msg/RobotGoal").expect("Hmm, can not create node");
    let simulator_cmd_publisher = node.create_publisher::<r2r::std_msgs::msg::String>("/simulator_command").expect("Hmm, can not create node");
    let resource_publisher = node.create_publisher::<r2r::sp_messages::msg::Resources>("/sp/resources").expect("Hmm, can not create node");

    let rx_out = channel::tick(Duration::from_millis(300));
    thread::spawn(move || loop {
        match rx_out.recv() {
            Ok(_) => {
                let x = state.lock().unwrap();
                
                let msg = x.get_value_from_cmd("/gripper").expect("hmm, no gripper in cmd").clone();
                gripper_publisher.publish(msg).expect("Could not send to gripper");

                let msg = x.get_value_from_cmd("/control_box").expect("hmm, no control_box in cmd").clone();
                control_box_publisher.publish(msg).expect("Could not send to control_box");

                let msg = x.get_value_from_cmd("/camera").expect("hmm, no camera in cmd").clone();
                camera_publisher.publish(msg).expect("Could not send to camera");

                let msg = x.get_value_from_cmd("/r1").expect("hmm, no r1 in cmd").clone();
                r1_publisher.publish(msg).expect("Could not send to r1");

                let msg = x.get_value_from_cmd("/r2").expect("hmm, no r2 in cmd").clone();
                r2_publisher.publish(msg).expect("Could not send to r2");

                let msg = x.get_value_from_cmd("/cubes").expect("hmm, no cubes in cmd").clone();
                let msg = r2r::std_msgs::msg::String { data: msg.to_string() };
                simulator_cmd_publisher.publish(&msg).expect("Could not send to r2");

                let msg = r2r::sp_messages::msg::Resources{resources: x.nodes.clone()};
                resource_publisher.publish(&msg).expect("Can not send resources");
            }
            Err(e) => {
                panic!("Send state channel out did not work: {:?}", e);
            }
        }
    });

}


fn listner(node: &mut r2r::Node, state: Arc<Mutex<API_State>>) {

        // callbacks
        let state_cb = state.to_owned();
        let gripper_cb = move |msg: r2r::Result<serde_json::Value>| {
            state_cb.lock().unwrap().upd_state("/gripper", msg.unwrap());
        };

        let state_cb = state.to_owned();
        let control_box_cb = move |msg: r2r::Result<serde_json::Value>| {
            state_cb.lock().unwrap().upd_state("/control_box", msg.unwrap());
        };

        let state_cb = state.to_owned();
        let camera_cb = move |msg: r2r::Result<serde_json::Value>| {
            state_cb.lock().unwrap().upd_state("/camera", msg.unwrap());
        };

        let state_cb = state.to_owned();
        let r1_cb = move |msg: r2r::Result<serde_json::Value>| {
            state_cb.lock().unwrap().upd_state("/r1", msg.unwrap());
        };

        let state_cb = state.to_owned();
        let r2_cb = move |msg: r2r::Result<serde_json::Value>| {
            state_cb.lock().unwrap().upd_state("/r2", msg.unwrap());
        };

        let state_cb = state.to_owned();
        let cubes_cb = move |msg: r2r::std_msgs::msg::String| {
            let json: serde_json::Value = serde_json::from_str(&msg.data).unwrap();
            state_cb.lock().unwrap().upd_state("/cubes", json);
        };

        let state_cb = state.to_owned();
        let resource_cb = move |msg: r2r::sp_messages::msg::RegisterResource| {
            let mut x = state_cb.lock().unwrap();
            let resource: String = msg.path.clone();
            if !x.nodes.contains(&resource) {
                x.nodes.push(resource);
            }
        };

        let state_cb = state.to_owned();
        let cmd_cb = move |msg: r2r::std_msgs::msg::String| {
            let mut x = state_cb.lock().unwrap();
            let json: serde_json::Value = serde_json::from_str(&msg.data).unwrap();
            x.cmd = json;
        };

        let _gripper = node.subscribe_untyped("/gripper/state", "gripper_msgs/msg/State", Box::new(gripper_cb));
        let _control_box = node.subscribe_untyped("/control_box/measured", "control_box_msgs/msg/Measured", Box::new(control_box_cb));
        let _camera = node.subscribe_untyped("/camera/measured", "camera_msgs/msg/Measured", Box::new(camera_cb));
        let _r1 = node.subscribe_untyped("/r1/measured", "robot_msgs/msg/RobotState", Box::new(r1_cb));
        let _r2 = node.subscribe_untyped("/r2/measured", "robot_msgs/msg/RobotState", Box::new(r2_cb));
        let _cubes = node.subscribe("/simulator_state",  Box::new(cubes_cb));
        let _resource = node.subscribe("/sp/resource",  Box::new(resource_cb));
        let _input = node.subscribe("cmd",  Box::new(cmd_cb));
    

}

// let state = serde_json::json!({
//     "r1": {
//         "act_pos": ""
//     },
//     "r2": {
//         "act_pos": ""
//     },
//     "control_box": {
//         "blue_light_on": false
//     },
//     "camera": {
//         "scanning": false,
//         "done": false,
//         "result": 0,
//     },
//     "gripper": {
//         "closed": false,
//         "part_sensor": false,
//     },
//     "cubes": {
//         "at_input_output_pos": false,
//     },
// });