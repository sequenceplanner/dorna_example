use r2r;
use failure::Error;
use std::cell::RefCell;
use std::rc::Rc;

use r2r::gripper_msgs::msg::{Goal, State};
use r2r::std_msgs::msg::String;

use serde_json;

mod resource_handler;

#[derive(PartialEq, Copy, Clone)]
enum GripperState {
    Open,
    Closed,
}

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "gripper_simulator", "")?;

    let state_publisher = node.create_publisher::<State>("state")?;

    let state = Rc::new(RefCell::new(GripperState::Open));
    let sim_state = Rc::new(RefCell::new(false));
    let namespace = node.namespace().unwrap();
    let initial_goal = Goal{close: false};
    let sp_comm = crate::resource_handler::ResourceHandler::new(
        &mut node,
        namespace,
        serde_json::to_string(&initial_goal).unwrap()
    ).unwrap();

    // goal callback
    let state_cb = state.to_owned();
    let nl_cb = node.logger().to_owned();
    let sp_goal_cb = move |msg: Goal| {
        let mut state = state_cb.borrow_mut();
        sp_comm.last_goal(serde_json::to_string(&msg).unwrap());

        if msg.close && *state == GripperState::Open {
            r2r::log_info!(&nl_cb, "closing the gripper");
            *state = GripperState::Closed;
        }
        else if !msg.close && (*state == GripperState::Closed) {
            r2r::log_info!(&nl_cb, "opening the gripper");
            *state = GripperState::Open;
        }
    };

    let _goal_sub = node.subscribe("goal", Box::new(sp_goal_cb))?;


    let sim_state_cb = sim_state.to_owned();
    let sim_cb = move |msg: String| {
        let str = msg.data.clone();
        let cubes = serde_json::from_str::<Vec<serde_json::Value>>(&msg.data);
        if let Ok(cubes) = cubes.as_ref() {
            *sim_state_cb.borrow_mut() = false;
            for c in cubes {
                if c.get("position") == Some(&serde_json::Value::String("/dorna/r1/dorna_5_link".into())) {
                    *sim_state_cb.borrow_mut() = true;
                }
            }
        }
    };

    let _sim_sub = node.subscribe("/simulator_state", Box::new(sim_cb))?;


    loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        let msg = match *state.borrow() {
            GripperState::Open => State { closed: false, part_sensor: *sim_state.borrow() },
            GripperState::Closed => State { closed: true, part_sensor: *sim_state.borrow() },
        };
        state_publisher.publish(&msg).unwrap();
    }
}
