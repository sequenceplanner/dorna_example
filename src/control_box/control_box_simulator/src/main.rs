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
    let state = Arc::new(Mutex::new(Measured::default()));

    // goal callback
    let state_task = state.clone();
    let state_publisher_task = state_publisher.clone();
    let nl_task = node.logger().to_owned();

    tokio::task::spawn(async move {
        loop {
            match goal_subscriber.next().await {
                Some(msg) => {
                    let mut state = state_task.lock().unwrap();
                    if state.blue_light_on != msg.blue_light {
                        state.blue_light_on = msg.blue_light;
                        r2r::log_info!(&nl_task, "blue light is {}",
                                       if state.blue_light_on { "on" } else { "off" });
                        state_publisher_task.publish(&state).expect("The node publisher is dead");
                    };

                    if state.conv_running_left && !state.conv_sensor {
                        state.conv_sensor = true;
                        r2r::log_info!(&nl_task, "part arrived");
                        state_publisher_task.publish(&state).expect("The node publisher is dead");
                    };
                    if state.conv_running_right && state.conv_sensor {
                        state.conv_sensor = false;
                        r2r::log_info!(&nl_task, "part left");
                        state_publisher_task.publish(&state).expect("The node publisher is dead");
                    };
                    if !state.conv_running_left && msg.conv_left && !msg.conv_right {
                        state.conv_running_left = true;
                        r2r::log_info!(&nl_task, "conveyor started LEFT");
                        state_publisher_task.publish(&state).expect("The node publisher is dead");
                    };
                    if !state.conv_running_right && !msg.conv_left && msg.conv_right {
                        state.conv_running_right = true;
                        r2r::log_info!(&nl_task, "conveyor started RIGHT");
                        state_publisher_task.publish(&state).expect("The node publisher is dead");
                    };
                    if (state.conv_running_left || state.conv_running_right) && !msg.conv_left && !msg.conv_right {
                        state.conv_running_left = false;
                        state.conv_running_right = false;
                        r2r::log_info!(&nl_task, "conveyor STOPPED");
                        state_publisher_task.publish(&state).expect("The node publisher is dead");
                    };
                }
                None => break,
            }
        }
    });

    tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        // publish the state once in a while
        state_publisher
            .publish(&state.lock().unwrap())
            .expect("The node publisher is dead");
    }).await?;

    Ok(())
}
