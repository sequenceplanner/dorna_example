use futures::executor::LocalPool;
use futures::stream::{Stream, StreamExt};
use futures::task::LocalSpawnExt;
use r2r;
use r2r::control_box_msgs::action::SetLight;
use r2r::control_box_msgs::msg::Measured;
use std::sync::{Arc, Mutex};

async fn set_light_server(
    state: Arc<Mutex<bool>>,
    mut requests: impl Stream<Item = r2r::GoalRequest<SetLight::Action>> + Unpin,
) {
    loop {
        match requests.next().await {
            Some(req) => {
                let (mut g, mut _cancel) = req.accept().expect("could not accept goal");

                *state.lock().unwrap() = g.goal.on;

                let result = SetLight::Result::default();

                println!("blue light changed to: {}", g.goal.on);

                g.succeed(result).expect("could not send result");
            }
            None => break,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "control_box_simulator", "/control_box")?;
    let state_publisher = node.create_publisher::<Measured>("measured")?;

    let server_requests = node.create_action_server::<SetLight::Action>("set_light")?;

    // state
    let blue_light: Arc<Mutex<bool>> = Arc::new(Mutex::new(false));

    spawner
        .spawn_local(set_light_server(blue_light.clone(), server_requests))
        .unwrap();

    loop {
        node.spin_once(std::time::Duration::from_millis(1000));
        pool.run_until_stalled();
        // publish the state once in a while
        let msg = Measured {
            blue_light_on: *blue_light.lock().unwrap(),
        };
        state_publisher
            .publish(&msg)
            .expect("The node publisher is dead");
    }
}
