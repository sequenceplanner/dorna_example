use r2r;
use futures::prelude::*;
use std::time::Duration;
use r2r::gripper_msgs::msg::{Goal, Measured};
use rand::Rng;
use std::sync::{Arc, Mutex};

#[derive(PartialEq, Copy, Clone)]
enum GripperState {
    Open,
    ClosedWithPart,
    ClosedWithoutPart
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "simulator", "gripper")?;

    let state_publisher = node.create_publisher::<Measured>("measured")?;
    let mut goal_subscriber = node.subscribe::<Goal>("goal")?;

    let state = Arc::new(Mutex::new(GripperState::Open));

    // goal callback
    let state_task = state.to_owned();
    let nl_task = node.logger().to_owned();

    tokio::task::spawn(async move {
        loop {
            match goal_subscriber.next().await {
                Some(msg) => {
                    if msg.close && *state_task.lock().unwrap() == GripperState::Open {
                        r2r::log_info!(&nl_task, "closing the gripper");

                        // take transition slowly so we see what's going on
                        tokio::time::sleep(Duration::from_millis(2000)).await;
                        // for now, always fail.
                        let fail = false; // rand::thread_rng().gen_range(0,5) < 1;
                        if fail {
                            r2r::log_warn!(&nl_task, "closed without a part");
                            *state_task.lock().unwrap() = GripperState::ClosedWithoutPart;
                        } else {
                            r2r::log_info!(&nl_task, "closed with a part");
                            *state_task.lock().unwrap() = GripperState::ClosedWithPart;
                        }
                    }
                    else if !msg.close && (*state_task.lock().unwrap() != GripperState::Open) {
                        r2r::log_info!(&nl_task, "opening the gripper");
                        tokio::time::sleep(Duration::from_millis(2000)).await;
                        r2r::log_info!(&nl_task, "opened the gripper");
                        *state_task.lock().unwrap() = GripperState::Open;
                    }
                }
                None => break,
            }
        }
    });

    tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        let state = state.lock().unwrap();
        let msg = match *state {
            GripperState::Open => Measured { closed: false, part_sensor: false },
            GripperState::ClosedWithPart => Measured { closed: true, part_sensor: true },
            GripperState::ClosedWithoutPart => Measured { closed: true, part_sensor: false },
        };
        state_publisher.publish(&msg).expect("The node publisher is dead.");
    }).await?;

    Ok(())
}
