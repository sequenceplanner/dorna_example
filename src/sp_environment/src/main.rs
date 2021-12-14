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
        .create_client::<r2r::sp_msgs::srv::Json::Service>("/env/sp/set_model")
        .unwrap();

    let client_state = node
        .create_client::<r2r::sp_msgs::srv::Json::Service>("/env/sp/set_state")
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

    let conv_left = Variable::new_boolean("goal/conv_left", VariableType::Measured);
    let conv_left = resource.add_variable(conv_left);

    let conv_right = Variable::new_boolean("goal/conv_right", VariableType::Measured);
    let conv_right = resource.add_variable(conv_right);

    let blue_light_on = Variable::new_boolean("measured/blue_light_on", VariableType::Command);
    let blue_light_on = resource.add_variable(blue_light_on);

    let conv_running_left = Variable::new_boolean("measured/conv_running_left", VariableType::Command);
    let conv_running_left = resource.add_variable(conv_running_left);

    let conv_running_right = Variable::new_boolean("measured/conv_running_right", VariableType::Command);
    let conv_running_right = resource.add_variable(conv_running_right);

    let conv_sensor = Variable::new_boolean("measured/conv_sensor", VariableType::Command);
    let conv_sensor = resource.add_variable(conv_sensor);

    resource.setup_ros_incoming("goal", &format!("/{}/goal", resource.path().leaf()),
                                MessageType::Ros("control_box_msgs/msg/Goal".into()),
    &[
        MessageVariable::new(&blue_light, "blue_light"),
        MessageVariable::new(&conv_left, "conv_left"),
        MessageVariable::new(&conv_right, "conv_right"),
    ]);

    resource.setup_ros_outgoing(
        "measured",
        &format!("/{}/measured",
        resource.path().leaf()),
        MessageType::Ros("control_box_msgs/msg/Measured".into()),
        &[
            MessageVariable::new(&blue_light_on, "blue_light_on"),
            MessageVariable::new(&conv_running_left, "conv_running_left"),
            MessageVariable::new(&conv_running_right, "conv_running_right"),
            MessageVariable::new(&conv_sensor, "conv_sensor"),
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

    let e_run_left_finish = Transition::new("run_left_finish",
                                            p!([p: conv_left] && [!p: conv_right] && [!p: conv_running_left] && [!p: conv_running_right]),
                                            vec![ a!( p: conv_running_left)], TransitionType::Runner);
    resource.add_transition(e_run_left_finish);

    // let e_run_right_finish = Transition::new("run_right_finish",
    //                                         p!([!p: conv_left] && [p: conv_right] && [!p: conv_running_left] && [!p: conv_running_right]),
    //                                         vec![ a!( p: conv_running_right)], TransitionType::Runner);
    // resource.add_transition(e_run_right_finish);
}

fn make_gripper(resource: &mut Resource) {
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
}

pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("environment");

    let t1 = "take1"; // shelf poses  // take1 = up
    let leave = "leave"; // down at conveyor // leave = pick

    let r1 = m.add_resource("/dorna/r1");
    make_dorna(m.get_resource(&r1), &[t1, leave]);

    let control_box = m.add_resource("control_box");
    make_control_box(m.get_resource(&control_box));

    let r1_gripper = m.add_resource("r1_gripper");
    make_gripper(m.get_resource(&r1_gripper));

    let r1_ap = m.get_resource(&r1).get_variable("measured/act_pos");
    let r1_rp = m.get_resource(&r1).get_variable("goal/ref_pos");
    let r1_moving = m.get_resource(&r1).get_predicate("moving");

    let cb_blue_light_on = m.get_resource(&control_box).get_variable("measured/blue_light_on");
    let cb_conv_run_left = m.get_resource(&control_box).get_variable("goal/conv_left");

    // hack
    let do_reset = m.get_resource(&control_box).get_variable("goal/conv_right");
    let is_reset = m.get_resource(&control_box).get_variable("measured/conv_running_right");

    let cb_conv_running_left = m.get_resource(&control_box).get_variable("measured/conv_running_left");
    let cb_conv_running_right = m.get_resource(&control_box).get_variable("measured/conv_running_right");
    let cb_conv_sensor = m.get_resource(&control_box).get_variable("measured/conv_sensor");

    let r1_gripper_part_sensor = m.get_resource(&r1_gripper).get_variable("measured/part_sensor");
    let r1_gripper_closed = m.get_resource(&r1_gripper).get_variable("measured/closed");
    let r1_gripper_close = m.get_resource(&r1_gripper).get_variable("goal/close");

    // the product state+transitions. this is what we are interested in learning.
    let buffer_domain = &[
        0.to_spvalue(), // buffer empty
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let conveyor = m.add_product_domain("conveyor", buffer_domain);
    let dorna_holding = m.add_product_domain("dorna_holding", buffer_domain);

    // conv running left => new part arrives, if there is no part already.
    let conv_left_sensor = Transition::new("run_left_finish",
                                           p!([p: conveyor == 0] && [p: cb_conv_running_left]),
                                           vec![ a!( p: conveyor = 1)], TransitionType::Runner);
    m.add_transition(conv_left_sensor);

    // conv stop running left
    let conv_left_stop = Transition::new("run_left_stop",
                                           p!([!p: cb_conv_run_left] && [p: cb_conv_running_left]),
                                           vec![ a!( ! p: cb_conv_running_left)], TransitionType::Runner);
    m.add_transition(conv_left_stop);


    // set sensor on conveyor based on product state.
    let set_conv_sensor = Transition::new("set_conv_finish",
                                           p!([p: conveyor != 0] && [!p: cb_conv_sensor]),
                                           vec![ a!( p: cb_conv_sensor)], TransitionType::Runner);
    m.add_transition(set_conv_sensor);
    let reset_conv_sensor = Transition::new("reset_conv_finish",
                                            p!([p: conveyor == 0] && [p: cb_conv_sensor]),
                                            vec![ a!( !p: cb_conv_sensor)], TransitionType::Runner);
    m.add_transition(reset_conv_sensor);

    let set_gripper_sensor = Transition::new("set_gripper_finish",
                                             p!([p: dorna_holding != 0] && [!p: r1_gripper_part_sensor]),
                                             vec![ a!( p: r1_gripper_part_sensor)], TransitionType::Runner);
    m.add_transition(set_gripper_sensor);
    let reset_gripper_sensor = Transition::new("reset_gripper_finish",
                                               p!([p: dorna_holding == 0] && [p: r1_gripper_part_sensor]),
                                               vec![ a!( !p: r1_gripper_part_sensor)], TransitionType::Runner);
    m.add_transition(reset_gripper_sensor);


    // gripper based on product state
    let e_finish_part = Transition::new("finish_part",
                                        p!([p: r1_gripper_close] && [!p: r1_gripper_closed] &&
                                           [p: r1_ap == leave] && [p: r1_rp == leave] &&
                                           [p: conveyor != 0]),
                                        vec![a!(p: r1_gripper_closed),
                                             a!(p: dorna_holding <- p:conveyor),
                                             a!(p: conveyor = 0)
                                        ],
                                        TransitionType::Runner);
    m.add_transition(e_finish_part);

    let e_finish_no_part = Transition::new("finish_no_part",
                                        p!([p: r1_gripper_close] && [!p: r1_gripper_closed] &&
                                           [! [[p: r1_ap == leave] && [p: r1_rp == leave] &&
                                               [p: conveyor != 0]]]),
                                        vec![a!(p: r1_gripper_closed),
                                             a!(p: dorna_holding = 0),
                                        ],
                                        TransitionType::Runner);
    m.add_transition(e_finish_no_part);

    let e_finish_open_conv = Transition::new("finish_open_conv",
                                             p!([!p: r1_gripper_close] && [p: r1_gripper_closed] &&
                                                [p: r1_ap == leave] && [p: r1_rp == leave] &&
                                                [p: conveyor == 0]),
                                             vec![a!(!p: r1_gripper_closed),
                                                  a!(p: conveyor <- p: dorna_holding),
                                                  a!(p: dorna_holding = 0)
                                             ],
                                             TransitionType::Runner);
    m.add_transition(e_finish_open_conv);

    let e_finish_open_away = Transition::new("finish_open_away",
                                             p!([!p: r1_gripper_close] && [p: r1_gripper_closed] &&
                                                [! [[p: r1_ap == leave] && [p: r1_rp == leave] &&
                                                    [p: conveyor == 0]]]),
                                             vec![a!(!p: r1_gripper_closed),
                                                  a!(p: dorna_holding = 0)
                                             ],
                                             TransitionType::Runner);
    m.add_transition(e_finish_open_away);

    let reset = Transition::new("reset_system",
                                p!([p: do_reset] && [!p: is_reset]),
                                vec![
                                    a!(p: conveyor = 0),
                                    a!(p: dorna_holding = 0),

                                     a!(p: r1_ap = t1),
                                     a!(p: r1_rp = t1),
                                     a!(p: cb_blue_light_on = false),
                                     a!(p: cb_conv_run_left = false),
                                     a!(p: cb_conv_running_left = false),
                                     a!(p: cb_conv_sensor = false),
                                     a!(p: r1_gripper_part_sensor = false),
                                     a!(p: r1_gripper_closed = true),
                                     a!(p: r1_gripper_close = true),

                                     a!(p: is_reset)
                                ],
                                TransitionType::Runner);
    m.add_transition(reset);

    let reset2 = Transition::new("reset_system2",
                                 p!([!p: do_reset] && [p: is_reset]),
                                 vec![a!(!p: is_reset)],
                                 TransitionType::Runner);
    m.add_transition(reset2);

    // let e_finish_no_part = Transition::new("finish_no_part", p!([p: closing]), vec![a!(p: closed), a!(!p: part_sensor)], TransitionType::Runner);
    // m.add_transition(e_finish_no_part);

    // let e_open_finish = Transition::new("open_finish", p!(p:opening), vec![ a!( !p: closed), a!( !p: part_sensor)], TransitionType::Runner);
    // m.add_transition(e_open_finish);

    // transitions are copied from the operations in the model.

    // let take = Transition::new("r3_take_conveyor",
    //                            p!([p: conveyor != 0] && [p: dorna_holding == 0] &&
    //                               [p: r1_ap == leave] && [!p: r1_moving]),
    //                            vec![ a!(p:dorna_holding <- p:conveyor), a!(p: conveyor = 0)],
    //                            TransitionType::Runner);
    // m.add_transition(take);

    // let leave = Transition::new("r3_leave_conveyor",
    //                             p!([p: dorna_holding != 0] && [p: conveyor == 0] &&
    //                                [p: r1_ap == leave] && [!p: r1_moving]),
    //                             vec![ a!(p:conveyor <- p:dorna_holding), a!(p: dorna_holding = 0)],
    //                             TransitionType::Runner);
    // m.add_transition(leave);

    let initial_state = SPState::new_from_values(&[
        // initial resource state
        (r1_ap, t1.to_spvalue()),
        (r1_rp, t1.to_spvalue()),

        (cb_blue_light_on, false.to_spvalue()),
        (cb_conv_run_left, false.to_spvalue()),
        (cb_conv_running_left, false.to_spvalue()),
        (cb_conv_running_right, false.to_spvalue()),
        (cb_conv_sensor, false.to_spvalue()),

        (r1_gripper_part_sensor, false.to_spvalue()),
        (r1_gripper_closed, true.to_spvalue()),
        (r1_gripper_close, true.to_spvalue()),

        // initial product state
        (dorna_holding, 0.to_spvalue()),
        (conveyor, 0.to_spvalue()),
    ]);

    return (m, initial_state);
}
