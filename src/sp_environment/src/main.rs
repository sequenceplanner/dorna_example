use sp_domain::*;

#[tokio::main]
async fn main(){
    let (model, state) = make_model();
    let state_json = SPStateJson::from_state_flat(&state)
        .to_json()
        .to_string();
    let compiled_model = sp_formal::CompiledModel::from(model.clone());
    let cm_json = serde_json::to_string(&compiled_model).unwrap();

    let state_req = r2r::sp_msgs::srv::Json::Request { json: state_json };
    let model_req = r2r::sp_msgs::srv::Json::Request { json: cm_json };

    let ctx = r2r::Context::create().map_err(SPError::from_any).unwrap();
    let mut node = r2r::Node::create(ctx, "model_maker", "")
        .map_err(SPError::from_any)
        .unwrap();

    let client = node
        .create_client::<r2r::sp_msgs::srv::Json::Service>("sp/set_model")
        .unwrap();
    let client_state = node
        .create_client::<r2r::sp_msgs::srv::Json::Service>("sp/set_state")
        .unwrap();

    let cav = node.is_available(&client).unwrap();
    let csv = node.is_available(&client_state).unwrap();

    let kill = std::sync::Arc::new(std::sync::Mutex::new(false));

    let k = kill.clone();
    let spin_handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
        if *k.lock().unwrap() {
            println!("Killing");
            break;
        }
    });

    print!("waiting for services...");
    let _r = tokio::join! {
        cav,
        csv,
    };
    println!("done");

    let d = std::time::Duration::from_secs(5);
    loop {
        print!("Sending state request... ");
        let result = tokio::time::timeout(d, client_state.request(&state_req).unwrap()).await;
        println!("{:?}", result);
        if result.is_ok() {
            break;
        }
    }
    loop {
        print!("Sending model change request... ");
        let result = tokio::time::timeout(d, client.request(&model_req).unwrap()).await;
        println!("{:?}", result);
        if result.is_ok() {
            break;
        }
    }

    *kill.lock().unwrap() = true;
    spin_handle.await.unwrap();
}

fn make_dorna(resource: &mut Resource, poses: &[&str]) {
    let mut domain = poses.iter().map(|p| p.to_spvalue()).collect::<Vec<SPValue>>();
    domain.push("unknown".to_spvalue());
    let ref_pos = Variable::new("goal/ref_pos", VariableType::Measured, SPValueType::String, domain.clone());
    let act_pos = Variable::new("measured/act_pos", VariableType::Measured, SPValueType::String, domain.clone());

    let ref_pos = resource.add_variable(ref_pos);
    let act_pos = resource.add_variable(act_pos);

    resource.setup_ros_incoming("goal", &format!("/dorna/{}/goal", resource.path().leaf()),
                                MessageType::Ros("robot_msgs/msg/RobotGoal".into()),
    &[
        MessageVariable::new(&ref_pos, "ref_pos")
    ]);
    resource.setup_ros_incoming("measured",&format!("/dorna/{}/measured", resource.path().leaf()),
                                MessageType::Ros("robot_msgs/msg/RobotState".into()),
    &[
        MessageVariable::new(&act_pos, "act_pos")
    ]);
}

fn make_control_box(resource: &mut Resource) {
    let blue_light = Variable::new_boolean("goal/blue_light", VariableType::Measured);
    let blue_light = resource.add_variable(blue_light);

    let blue_light_on = Variable::new_boolean("measured/blue_light_on", VariableType::Measured);
    let blue_light_on = resource.add_variable(blue_light_on);

    resource.setup_ros_incoming("goal", &format!("/{}/goal", resource.path().leaf()),
                                MessageType::Ros("control_box_msgs/msg/Goal".into()),
    &[
        MessageVariable::new(&blue_light, "blue_light")
    ]);

    resource.setup_ros_incoming(
        "measured",
        &format!("/{}/measured",
        resource.path().leaf()),
        MessageType::Ros("control_box_msgs/msg/Measured".into()),
        &[
            MessageVariable::new(&blue_light_on, "blue_light_on")
        ]
    );
}

fn make_camera(resource: &mut Resource) {
    let do_scan = Variable::new_boolean("goal/do_scan", VariableType::Measured);
    let scanning = Variable::new_boolean("measured/scanning", VariableType::Measured);
    let done = Variable::new_boolean("measured/done", VariableType::Measured);

    let domain = vec![0,1,2,3].iter().map(|v|v.to_spvalue()).collect();
    let result = Variable::new("measured/result", VariableType::Measured, SPValueType::Int32, domain);

    let do_scan = resource.add_variable(do_scan);
    let scanning = resource.add_variable(scanning);
    let done = resource.add_variable(done);
    let result = resource.add_variable(result);

    resource.setup_ros_incoming("goal", &format!("/{}/goal", resource.path().leaf()),
                                MessageType::Ros("camera_msgs/msg/Goal".into()),
    &[
        MessageVariable::new(&do_scan, "do_scan")
    ]);
    resource.setup_ros_incoming("measured", &format!("/{}/measured", resource.path().leaf()),
                                MessageType::Ros("camera_msgs/msg/Measured".into()),
    &[
        MessageVariable::new(&scanning, "scanning"),
        MessageVariable::new(&done, "done"),
        MessageVariable::new(&result, "result"),
    ]);
}

fn make_gripper_fail(resource: &mut Resource) {
    let close = Variable::new_boolean("goal/close", VariableType::Measured);
    let closed = Variable::new_boolean("measured/closed", VariableType::Measured);
    let part_sensor = Variable::new_boolean("measured/part_sensor", VariableType::Measured);

    let domain: Vec<_> = vec![0,1,2].iter().map(|v|v.to_spvalue()).collect();

    let close = resource.add_variable(close);
    let closed = resource.add_variable(closed);
    let part_sensor = resource.add_variable(part_sensor);

    resource.setup_ros_incoming("goal", &format!("/{}/goal", resource.path().leaf()),
                                MessageType::Ros("gripper_msgs/msg/Goal".into()),
    &[
        MessageVariable::new(&close, "close")
    ]);
    resource.setup_ros_incoming("measured", &format!("/{}/measured", resource.path().leaf()),
                                MessageType::Ros("gripper_msgs/msg/Measured".into()),
    &[
        MessageVariable::new(&closed, "closed"),
        MessageVariable::new(&part_sensor, "part_sensor"),
    ]);
}

pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("environment");

    let pt = "pre_take";
    let scan = "scan";
    let t1 = "take1"; // shelf poses
    let t2 = "take2";
    let t3 = "take3";
    let leave = "leave"; // down at conveyor

    let r1 = m.add_resource("/dorna/r1");
    make_dorna(m.get_resource(&r1), &[pt, scan, t1, t2, t3, leave]);

    let r2 = m.add_resource("/dorna/r2");
    make_dorna(m.get_resource(&r2), &[pt, scan, t1, t2, t3, leave]);

    let control_box = m.add_resource("control_box");
    make_control_box(m.get_resource(&control_box));

    let camera = m.add_resource("camera");
    make_camera(m.get_resource(&camera));

    let gripper = m.add_resource("gripper");
    make_gripper_fail(m.get_resource(&gripper));

    let initial_state = SPState::new_from_values(&[
    ]);

    return (m, initial_state);
}
