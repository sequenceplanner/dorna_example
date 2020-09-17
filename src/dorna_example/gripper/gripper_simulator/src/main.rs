use r2r;
use failure::Error;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use r2r::gripper_msgs::msg::{Goal, State};

use rand::Rng;

mod resource_handler;

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

    let state = Rc::new(RefCell::new(GripperState::Open));
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

            // take transition slowly so we see what's going on
            std::thread::sleep(Duration::from_millis(2000));
            let fail = rand::thread_rng().gen_range(0,5) < 1;
            if fail {
                r2r::log_warn!(&nl_cb, "closed without a part");
                *state = GripperState::ClosedWithoutPart;
            } else {
                r2r::log_info!(&nl_cb, "closed with a part");
                *state = GripperState::ClosedWithPart;
            }
        }
        else if !msg.close && (*state != GripperState::Open) {
            r2r::log_info!(&nl_cb, "opening the gripper");
            std::thread::sleep(Duration::from_millis(2000));
            r2r::log_info!(&nl_cb, "opened the gripper");
            *state = GripperState::Open;
        }
    };

    let _goal_sub = node.subscribe("goal", Box::new(sp_goal_cb))?;


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
