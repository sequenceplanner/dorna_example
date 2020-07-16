use r2r;
use failure::Error;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use r2r::gripper_msgs::msg::{Goal, State};
use r2r::sp_messages::msg::{NodeCmd, NodeMode};

use rand::Rng;

#[derive(PartialEq, Copy, Clone)]
enum GripperState {
    Open,
    ClosedWithPart,
    ClosedWithoutPart
}

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "gripper_simulator", "")?;

    let state_publisher = node.create_publisher::<State>("state")?;
    let mode_publisher = node.create_publisher::<NodeMode>("mode")?;
    let last_seen_goal = Rc::new(RefCell::new(Goal::default()));
    let node_mode = Rc::new(RefCell::new(NodeMode { mode: "init".into(), echo: "".into() }));
    let state = Rc::new(RefCell::new(GripperState::Open));

    // goal callback
    let last_seen_goal_cb = last_seen_goal.clone();
    let state_cb = state.to_owned();
    let nl_cb = node.logger().to_owned();
    let sp_goal_cb = move |msg: Goal| {
        if *last_seen_goal_cb.borrow() == msg {
            return ;
        }
        last_seen_goal_cb.replace(msg);

        let close = last_seen_goal_cb.borrow().close;
        let state = *state_cb.borrow();
        if close && state == GripperState::Open {
            r2r::log_info!(&nl_cb, "closing the gripper");

            // take transition slowly so we see what's going on
            std::thread::sleep(Duration::from_millis(2000));
            let fail = rand::thread_rng().gen_range(0,5) < 2;
            if fail {
                r2r::log_warn!(&nl_cb, "closed without a part");
                state_cb.replace(GripperState::ClosedWithoutPart);
            } else {
                r2r::log_info!(&nl_cb, "closed with a part");
                state_cb.replace(GripperState::ClosedWithPart);
            }
        }
        else if !close && (state != GripperState::Open) {
            r2r::log_info!(&nl_cb, "opening the gripper");
            std::thread::sleep(Duration::from_millis(2000));
            r2r::log_info!(&nl_cb, "opened the gripper");
            state_cb.replace(GripperState::Open);
        }
    };

    let _goal_sub = node.subscribe("goal", Box::new(sp_goal_cb))?;

    // sp node mode callback
    let last_seen_goal_cb = last_seen_goal.clone();
    let mode_publisher_cb = mode_publisher.clone();
    let node_mode_cb = node_mode.clone();
    let sp_mode_cb = move |msg: NodeCmd| {
        let new_mode = if msg.mode == "run" {
            String::from("running")
        } else {
            String::from("init")
        };

        let echo = serde_json::to_string(&*last_seen_goal_cb.borrow()).unwrap();
        node_mode_cb.replace(NodeMode { echo, mode: new_mode});
        mode_publisher_cb.publish(&node_mode_cb.borrow()).unwrap();
    };

    let _node_sub = node.subscribe("node_cmd", Box::new(sp_mode_cb))?;

    loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        let state = *state.borrow();
        let msg = match state {
            GripperState::Open => State { closed: false, part_sensor: false },
            GripperState::ClosedWithPart => State { closed: true, part_sensor: true },
            GripperState::ClosedWithoutPart => State { closed: true, part_sensor: false },
        };
        state_publisher.publish(&msg).unwrap();
    }
}
