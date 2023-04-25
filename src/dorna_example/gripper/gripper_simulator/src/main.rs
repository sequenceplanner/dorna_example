use r2r;
use futures::{StreamExt, future};
use std::time::Duration;
use r2r::gripper_msgs::srv::{Open, Close, GetState};
use rand::Rng;
use std::sync::{Arc, Mutex};

#[derive(PartialEq, Copy, Clone)]
enum GripperState {
    Open,
    ClosedWithPart,
    ClosedWithoutPart
}


fn handle_get_state(shared_state: Arc<Mutex<GripperState>>, r: r2r::ServiceRequest<GetState::Service>) {
    let state = *shared_state.lock().unwrap();
    let resp = match state {
        GripperState::Open => GetState::Response { is_closed: false, has_part: false },
        GripperState::ClosedWithPart => GetState::Response { is_closed: true, has_part: true },
        GripperState::ClosedWithoutPart => GetState::Response { is_closed: true, has_part: false },
    };
    let _res = r.respond(resp);
}

async fn handle_close(nl: String, shared_state: Arc<Mutex<GripperState>>, r: r2r::ServiceRequest<Close::Service>) {
    let state = *shared_state.lock().unwrap();
    let new_state = if state == GripperState::Open {
        r2r::log_info!(&nl, "closing the gripper");

        // take transition slowly so we see what's going on
        tokio::time::sleep(Duration::from_millis(2000)).await;
        let fail = rand::thread_rng().gen_range(0,5) < 1;
        if fail {
            r2r::log_warn!(&nl, "closed without a part");
            GripperState::ClosedWithoutPart
        } else {
            r2r::log_info!(&nl, "closed with a part");
            GripperState::ClosedWithPart
        }
    } else {
        state
    };

    *shared_state.lock().unwrap() = new_state;

    let resp = Close::Response {
        has_part: new_state == GripperState::ClosedWithPart
    };
    let _res = r.respond(resp);
}

async fn handle_open(nl: String, shared_state: Arc<Mutex<GripperState>>, r: r2r::ServiceRequest<Open::Service>) {
    let state = *shared_state.lock().unwrap();
    let new_state = if state != GripperState::Open {
        r2r::log_info!(&nl, "opening the gripper");
        tokio::time::sleep(Duration::from_millis(2000)).await;
        r2r::log_info!(&nl, "opened the gripper");
        GripperState::Open
    } else {
        state
    };

    *shared_state.lock().unwrap() = new_state;

    let resp = Open::Response::default();
    let _res = r.respond(resp);
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "gripper_simulator", "")?;
    let nl = node.logger().to_string();

    let state = Arc::new(Mutex::new(GripperState::Open));

    let open_service = node.create_service::<Open::Service>("open")?;
    let close_service = node.create_service::<Close::Service>("close")?;
    let get_state_service = node.create_service::<GetState::Service>("get_state")?;
    let statec = state.clone();
    let nlc = nl.clone();
    tokio::task::spawn(async move {
        open_service.for_each(move |req| {
            handle_open(nlc.clone(), statec.clone(), req)
        }).await;
    });

    let statec = state.clone();
    let nlc = nl.clone();
    tokio::task::spawn(async move {
        close_service.for_each(move |req| {
            handle_close(nlc.clone(), statec.clone(), req)
        }).await;
    });

    let statec = state.clone();
    tokio::task::spawn(async move {
        get_state_service.for_each(move |req| {
            handle_get_state(statec.clone(), req);
            future::ready(())
        }).await;
    });

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}
