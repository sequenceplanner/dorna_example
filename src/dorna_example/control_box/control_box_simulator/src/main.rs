use r2r;
use futures::prelude::*;
use r2r::control_box_msgs::msg::{Goal, Measured};
use std::sync::{Arc, Mutex};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "simulator", "control_box")?;
    let state_publisher = node.create_publisher::<Measured>("measured")?;
    let mut goal_subscriber = node.subscribe::<Goal>("goal")?;

    // state of the light
    let state = Arc::new(Mutex::new(false));

    // goal callback
    let state_task = state.clone();
    let state_publisher_task = state_publisher.clone();
    let nl_task = node.logger().to_owned();

    tokio::task::spawn(async move {
        loop {
            match goal_subscriber.next().await {
                Some(msg) => {
                    let mut state = state_task.lock().unwrap();
                    if *state == msg.blue_light {
                        continue;
                    };

                    *state = msg.blue_light;
                    r2r::log_info!(&nl_task, "blue light is {}",
                                   if *state { "on" } else { "off" });

                    let msg = Measured {
                        blue_light_on: *state,
                    };
                    state_publisher_task.publish(&msg).expect("The node publisher is dead");
                }
                None => break,
            }
        }
    });

    tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        // publish the state once in a while
        let msg = Measured {
            blue_light_on: *state.lock().unwrap(),
        };
        state_publisher
            .publish(&msg)
            .expect("The node publisher is dead");
    }).await?;

    Ok(())
}
