use r2r;
use failure::Error;
use std::cell::RefCell;
use std::rc::Rc;

use r2r::control_box_msgs::msg::{Goal, Measured};

mod resource_handler;

fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "control_box_simulator", "/control_box")?;
    let state_publisher = node.create_publisher::<Measured>("measured")?;
    let namespace = node.namespace().unwrap();
    let initial_goal = Goal{blue_light: false};
    let sp_comm = crate::resource_handler::ResourceHandler::new(
        &mut node, 
        namespace,
        serde_json::to_string(&initial_goal).unwrap()
    ).unwrap();

    // state
    let blue_light: Rc<RefCell<bool>> = Rc::new(RefCell::new(false));

    // goal callback
    let state_publisher_cb = state_publisher.clone();
    let nl_cb = node.logger().to_string();
    let b_cb = blue_light.clone();
    let sp_goal_cb = move |msg: Goal| {
        let mut b = b_cb.borrow_mut();
        if *b == msg.blue_light && sp_comm.has_last_goal() {
            return;
        };

        *b = msg.blue_light;
        sp_comm.last_goal(serde_json::to_string(&msg).unwrap());
        r2r::log_info!(&nl_cb, "blue light is {}",
                       if *b { "on" } else { "off" });

        let msg = Measured { blue_light_on: *b };
        state_publisher_cb.publish(&msg).unwrap();
    };
    let _goal_sub = node.subscribe("goal", Box::new(sp_goal_cb))?;

    loop {
        node.spin_once(std::time::Duration::from_millis(1000));

        // publish the state once in a while
        let msg = Measured { blue_light_on: *blue_light.borrow() };
        state_publisher.publish(&msg).expect("The node publisher is dead");
    }
}
