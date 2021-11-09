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

    println!("waiting for services...");
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
    let act_pos = Variable::new("measured/act_pos", VariableType::Command, SPValueType::String, domain.clone());

    let ref_pos = resource.add_variable(ref_pos);
    let act_pos = resource.add_variable(act_pos);

    resource.setup_ros_incoming("goal", &format!("/dorna/{}/goal", resource.path().leaf()),
                                MessageType::Ros("robot_msgs/msg/RobotGoal".into()),
    &[
        MessageVariable::new(&ref_pos, "ref_pos")
    ]);
    resource.setup_ros_outgoing("measured",&format!("/dorna/{}/measured", resource.path().leaf()),
                                MessageType::Ros("robot_msgs/msg/RobotState".into()),
    &[
        MessageVariable::new(&act_pos, "act_pos")
    ]);

    let moving = Variable::new_predicate("moving", p!(p: ref_pos <!> p:act_pos));
    let moving = resource.add_variable(moving);

    let e_move_finish = Transition::new("move_finish", p!(p: moving), vec![ a!( p: act_pos <- p: ref_pos)], TransitionType::Runner);
    resource.add_transition(e_move_finish);
}

fn make_control_box(resource: &mut Resource) {
    let blue_light = Variable::new_boolean("goal/blue_light", VariableType::Measured);
    let blue_light = resource.add_variable(blue_light);

    let blue_light_on = Variable::new_boolean("measured/blue_light_on", VariableType::Command);
    let blue_light_on = resource.add_variable(blue_light_on);

    resource.setup_ros_incoming("goal", &format!("/{}/goal", resource.path().leaf()),
                                MessageType::Ros("control_box_msgs/msg/Goal".into()),
    &[
        MessageVariable::new(&blue_light, "blue_light")
    ]);

    resource.setup_ros_outgoing(
        "measured",
        &format!("/{}/measured",
        resource.path().leaf()),
        MessageType::Ros("control_box_msgs/msg/Measured".into()),
        &[
            MessageVariable::new(&blue_light_on, "blue_light_on")
        ]
    );

    // these transitions are the "inverse" of what we have specified as effect transitions
    // in the sp control model
    let e_blue_on_finish = Transition::new("blue_on_finish", p!([!p: blue_light_on] && [p: blue_light]),
                                           vec![ a!( p: blue_light_on)], TransitionType::Runner);
    resource.add_transition(e_blue_on_finish);

    let e_blue_off_finish = Transition::new("blue_off_finish", p!([p: blue_light_on] && [!p: blue_light]),
                                           vec![ a!( !p: blue_light_on)], TransitionType::Runner);
    resource.add_transition(e_blue_off_finish);
}

fn make_camera(resource: &mut Resource) {
    let do_scan = Variable::new_boolean("goal/do_scan", VariableType::Command);
    let scanning = Variable::new_boolean("measured/scanning", VariableType::Command);
    let done = Variable::new_boolean("measured/done", VariableType::Command);

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
    resource.setup_ros_outgoing("measured", &format!("/{}/measured", resource.path().leaf()),
                                MessageType::Ros("camera_msgs/msg/Measured".into()),
    &[
        MessageVariable::new(&scanning, "scanning"),
        MessageVariable::new(&done, "done"),
        MessageVariable::new(&result, "result"),
    ]);

    let started = Variable::new_predicate("started", p!([p: do_scan] && [!p: scanning]));
    let started = resource.add_variable(started);

    let executing = Variable::new_predicate("executing", p!([p: do_scan] && [p: scanning] && [!p: done]));
    let executing = resource.add_variable(executing);

    let resetting = Variable::new_predicate("resetting", p!([!p: do_scan] && [p: scanning] && [p: done]));
    let resetting = resource.add_variable(resetting);

    let e_starting = Transition::new("starting", p!([p: started]), vec![a!(p: scanning), a!(!p: done)],
                                     TransitionType::Runner);
    resource.add_transition(e_starting);

    let random = Action::new(result.clone(), Compute::Random(4));
    let e_finish = Transition::new("finish", p!([p: executing]), vec![a!(p: done), a!(p: scanning), random ],
                                   TransitionType::Runner);
    resource.add_transition(e_finish);

    let e_reset_effect = Transition::new("reset_effect", p!([p: resetting]), vec![a!(!p: done), a!(!p: scanning), a!(p: result = 0)], TransitionType::Runner);
    resource.add_transition(e_reset_effect);
}

fn make_gripper_fail(resource: &mut Resource) {
    let close = Variable::new_boolean("goal/close", VariableType::Measured);
    let closed = Variable::new_boolean("measured/closed", VariableType::Measured);
    let part_sensor = Variable::new_boolean("measured/part_sensor", VariableType::Measured);

    let close = resource.add_variable(close);
    let closed = resource.add_variable(closed);
    let part_sensor = resource.add_variable(part_sensor);

    resource.setup_ros_incoming("goal", &format!("/{}/goal", resource.path().leaf()),
                                MessageType::Ros("gripper_msgs/msg/Goal".into()),
    &[
        MessageVariable::new(&close, "close")
    ]);
    resource.setup_ros_outgoing("measured", &format!("/{}/measured", resource.path().leaf()),
                                MessageType::Ros("gripper_msgs/msg/Measured".into()),
    &[
        MessageVariable::new(&closed, "closed"),
        MessageVariable::new(&part_sensor, "part_sensor"),
    ]);


    let opening = Variable::new_predicate("opening", p!([!p: close] && [p: closed]));
    let opening = resource.add_variable(opening);

    let closing = Variable::new_predicate("closing", p!([p: close] && [!p: closed]));
    let closing = resource.add_variable(closing);

    let e_finish_part = Transition::new("finish_part", p!([p: closing]), vec![a!(p: closed), a!(p: part_sensor)], TransitionType::Runner);
    resource.add_transition(e_finish_part);
    let e_finish_no_part = Transition::new("finish_no_part", p!([p: closing]), vec![a!(p: closed), a!(!p: part_sensor)], TransitionType::Runner);
    resource.add_transition(e_finish_no_part);

    let e_open_finish = Transition::new("open_finish", p!(p:opening), vec![ a!( !p: closed), a!( !p: part_sensor)], TransitionType::Runner);
    resource.add_transition(e_open_finish);
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

    let r1_ap = m.get_resource(&r1).get_variable("measured/act_pos");
    let r2_ap = m.get_resource(&r2).get_variable("measured/act_pos");

    let camera_done = m.get_resource(&camera).get_variable("measured/done");
    let camera_result = m.get_resource(&camera).get_variable("measured/result");
    let camera_scanning = m.get_resource(&camera).get_variable("measured/scanning");

    let cb_blue_light_on = m.get_resource(&control_box).get_variable("measured/blue_light_on");

    let gripper_part_sensor = m.get_resource(&gripper).get_variable("measured/part_sensor");
    let gripper_closed = m.get_resource(&gripper).get_variable("measured/closed");

    let initial_state = SPState::new_from_values(&[
        (r1_ap, pt.to_spvalue()),
        (r2_ap, pt.to_spvalue()),
        (camera_done, false.to_spvalue()),
        (camera_result, 0.to_spvalue()),
        (camera_scanning, false.to_spvalue()),
        (cb_blue_light_on, false.to_spvalue()),
        (gripper_part_sensor, false.to_spvalue()),
        (gripper_closed, false.to_spvalue()),
    ]);

    return (m, initial_state);
}
