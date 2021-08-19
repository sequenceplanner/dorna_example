use sp_runner::*;
use sp_domain::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (model, initial_state) = make_model();
    launch_model(model, initial_state).await?;
    Ok(())
}

fn make_dorna(resource: &mut Resource, poses: &[&str]) {
    let mut domain = poses.iter().map(|p| p.to_spvalue()).collect::<Vec<SPValue>>();
    domain.push("unknown".to_spvalue());
    let ref_pos = Variable::new("goal/ref_pos", VariableType::Command, SPValueType::String, domain.clone());
    let act_pos = Variable::new("measured/act_pos", VariableType::Measured, SPValueType::String, domain.clone());
    let prev_pos = Variable::new("prev_pos", VariableType::Estimated, SPValueType::String, domain.clone());

    let ref_pos = resource.add_variable(ref_pos);
    let act_pos = resource.add_variable(act_pos);
    let prev_pos = resource.add_variable(prev_pos);

    let moving = Variable::new_predicate("moving", p!(p: ref_pos <!> p:act_pos));
    let moving = resource.add_variable(moving);

    let c_move_start = Transition::new("move_start", p!(! p: moving), vec![ a!( p: prev_pos <- p: act_pos), a!(p: ref_pos?)], TransitionType::Controlled);
    resource.add_transition(c_move_start);

    let e_move_finish = Transition::new("move_finish", p!(p: moving), vec![ a!( p: act_pos <- p: ref_pos)], TransitionType::Effect);
    resource.add_transition(e_move_finish);

    resource.setup_ros_outgoing("goal", &format!("/dorna/{}/goal", resource.path().leaf()), "robot_msgs/msg/RobotGoal");
    resource.setup_ros_incoming("measured",&format!("/dorna/{}/measured", resource.path().leaf()), "robot_msgs/msg/RobotState");
}

fn make_control_box(resource: &mut Resource) {
    let blue_light = Variable::new_boolean("goal/blue_light", VariableType::Command);
    let blue_light_on = Variable::new_boolean("measured/blue_light_on", VariableType::Measured);

    let blue_light = resource.add_variable(blue_light);
    let blue_light_on = resource.add_variable(blue_light_on);

    let c_blue_on_start = Transition::new("blue_on_start", p!(! p: blue_light_on), vec![ a!( p: blue_light)], TransitionType::Controlled);
    resource.add_transition(c_blue_on_start);

    let e_blue_on_finish = Transition::new("blue_on_finish", p!([p: blue_light] && [!p: blue_light_on]), vec![ a!( p: blue_light_on)], TransitionType::Effect);
    resource.add_transition(e_blue_on_finish);

    let c_blue_off_start = Transition::new("blue_off_start", p!(p: blue_light_on), vec![ a!( !p: blue_light)], TransitionType::Controlled);
    resource.add_transition(c_blue_off_start);

    let e_blue_off_finish = Transition::new("blue_off_finish", p!([!p: blue_light] && [p: blue_light_on]), vec![ a!( !p: blue_light_on)], TransitionType::Effect);
    resource.add_transition(e_blue_off_finish);

    resource.setup_ros_outgoing("goal",&format!("{}/goal", resource.path().leaf()), "control_box_msgs/msg/Goal");
    resource.setup_ros_incoming("measured",&format!("{}/measured", resource.path().leaf()), "control_box_msgs/msg/Measured");
}

fn make_camera(resource: &mut Resource) {
    let do_scan = Variable::new_boolean("goal/do_scan", VariableType::Command);
    let scanning = Variable::new_boolean("measured/scanning", VariableType::Measured);
    let done = Variable::new_boolean("measured/done", VariableType::Measured);

    let domain = vec![0,1,2,3].iter().map(|v|v.to_spvalue()).collect();
    let result = Variable::new("measured/result", VariableType::Measured, SPValueType::Int32, domain);

    let do_scan = resource.add_variable(do_scan);
    let scanning = resource.add_variable(scanning);
    let done = resource.add_variable(done);
    let result = resource.add_variable(result);

    let enabled = Variable::new_predicate("enabled", p!([!p: do_scan] && [!p: scanning]));
    let enabled = resource.add_variable(enabled);

    let started = Variable::new_predicate("started", p!([p: do_scan] && [!p: scanning]));
    let started = resource.add_variable(started);

    let executing = Variable::new_predicate("executing", p!([p: do_scan] && [p: scanning] && [!p: done]));
    let executing = resource.add_variable(executing);

    let finished = Variable::new_predicate("finished", p!([p: do_scan] && [p: scanning] && [p: done]));
    let finished = resource.add_variable(finished);

    let resetting = Variable::new_predicate("resetting", p!([!p: do_scan] && [p: scanning] && [p: done]));
    let resetting = resource.add_variable(resetting);


    let c_start = Transition::new("start", p!(p:enabled), vec![ a!( p: do_scan)], TransitionType::Controlled);
    resource.add_transition(c_start);

    let e_starting = Transition::new("starting", p!([p: started]), vec![a!(p: scanning), a!(!p: done)], TransitionType::Effect);
    resource.add_transition(e_starting);

    let e_finish0 = Transition::new("finish_0", p!([p: executing]), vec![a!(p: done), a!(p: scanning), a!(p: result = 0)], TransitionType::Effect); resource.add_transition(e_finish0);
    let e_finish1 = Transition::new("finish_1", p!([p: executing]), vec![a!(p: done), a!(p: scanning), a!(p: result = 1)], TransitionType::Effect); resource.add_transition(e_finish1);
    let e_finish2 = Transition::new("finish_2", p!([p: executing]), vec![a!(p: done), a!(p: scanning), a!(p: result = 2)], TransitionType::Effect); resource.add_transition(e_finish2);
    let e_finish3 = Transition::new("finish_3", p!([p: executing]), vec![a!(p: done), a!(p: scanning), a!(p: result = 3)], TransitionType::Effect); resource.add_transition(e_finish3);

    let c_reset = Transition::new("reset", p!([p: finished]), vec![a!(!p: do_scan)], TransitionType::Controlled);
    resource.add_transition(c_reset);

    let e_reset_effect = Transition::new("reset_effect", p!([p: resetting]), vec![a!(!p: done), a!(!p: scanning), a!(p: result = 0)], TransitionType::Effect);
    resource.add_transition(e_reset_effect);

    // modeling specification...
    let state_does_not_exist =
        Specification::new_transition_invariant("state_does_not_exist",
                                                p!(! [[!p: do_scan] && [p: scanning] && [!p: done]]));

    resource.add_specification(state_does_not_exist);

    resource.setup_ros_outgoing("goal", &format!("{}/goal", resource.path().leaf()), "camera_msgs/msg/Goal");
    resource.setup_ros_incoming("measured", &format!("{}/measured", resource.path().leaf()), "camera_msgs/msg/Measured");
}

fn make_gripper_fail(resource: &mut Resource) {

    let is_closed = resource.add_variable(Variable::new_boolean("get_state/is_closed", VariableType::Measured));
    let has_part = resource.add_variable(Variable::new_boolean("get_state/has_part", VariableType::Measured));
    let get = resource.add_variable( Variable::new_boolean("get_state/get", VariableType::Command));
    let close = resource.add_variable( Variable::new_boolean("get_state/close", VariableType::Command));
    let open = resource.add_variable( Variable::new_boolean("get_state/open", VariableType::Command));

    resource.setup_ros_service(
        "get_state",
        &format!("{}/get_state", resource.path().leaf()), 
        "gripper_msgs/srv/GetState", 
        p!(p: get),
        &[], 
        &[
            MessageVariable::new(&is_closed, "is_closed"), 
            MessageVariable::new(&has_part, "has_part")
            ] 
    );
    resource.setup_ros_service(
        "open",
        &format!("{}/open", resource.path().leaf()), 
        "gripper_msgs/srv/Open", 
        p!(p: open),
        &[], 
        &[] 
    );
    resource.setup_ros_service(
        "close",
        &format!("{}/close", resource.path().leaf()), 
        "gripper_msgs/srv/Close", 
        p!(p: close),
        &[], 
        &[MessageVariable::new(&has_part, "has_part")] 
    );



    let domain = vec![0,1,2].iter().map(|v|v.to_spvalue()).collect();
    let fail_count = Variable::new("fail_count", VariableType::Estimated, SPValueType::Int32, domain);
    let fail_count = resource.add_variable(fail_count);


    let c_close_start0 = Transition::new("close_start0", p!([!p:is_closed] && [p: fail_count == 0]), vec![ a!( p: close), a!( p: fail_count = 1)], TransitionType::Controlled);
    resource.add_transition(c_close_start0);
    let c_close_start1 = Transition::new("close_start1", p!([!p:is_closed] && [p: fail_count == 1]), vec![ a!( p: close), a!( p: fail_count = 2)], TransitionType::Controlled);
    resource.add_transition(c_close_start1);

    let done_closing = Transition::new("done_closing", p!([p: is_closed] && [p: close]), vec![a!(!p: close)], TransitionType::Auto);
    resource.add_transition(done_closing);

    let e_finish_part = Transition::new("finish_part", p!([p: close] && [!p: is_closed]), vec![a!(p: is_closed), a!(p: has_part)], TransitionType::Effect);
    resource.add_transition(e_finish_part);
    let e_finish_no_part = Transition::new("finish_no_part", p!([p: close] && [!p: is_closed]), vec![a!(p: is_closed), a!(!p: has_part)], TransitionType::Effect);
    resource.add_transition(e_finish_no_part);

    let a_close_finished_reset_fc = Transition::new("close_finished_reset_fc", p!([p: is_closed] && [p: has_part] && [p: fail_count != 0]), vec![a!(p: fail_count = 0)], TransitionType::Auto);
    resource.add_transition(a_close_finished_reset_fc);

    let c_open_start = Transition::new("open_start", p!(p:is_closed), vec![ a!( p: open)], TransitionType::Controlled);
    resource.add_transition(c_open_start);

    let e_open_finish = Transition::new("open_finish", p!([p: open] && [p: is_closed]), vec![ a!( !p: is_closed), a!( !p: has_part)], TransitionType::Effect);
    resource.add_transition(e_open_finish);

    let done_opening = Transition::new("done_opening", p!([!p: is_closed] && [p: open]), vec![a!(!p: open)], TransitionType::Auto);
    resource.add_transition(done_opening);

    let state_does_not_exist =
        Specification::new_transition_invariant("state_does_not_exist",
                                                p!(! [[!p: is_closed] && [p: has_part]]));
    resource.add_specification(state_does_not_exist);

    //resource.setup_ros_outgoing(&format!("{}/goal", resource.path().leaf()), "gripper_msgs/msg/Goal");
    //resource.setup_ros_incoming(&format!("{}/state", resource.path().leaf()), "gripper_msgs/msg/State");

    // testing service. The variabletype does bot matter

}

pub fn make_model() -> (Model, SPState) {
    let mut m = Model::new("cylinders2"); // TODO: fix in python world...

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

    let buffer_domain = &[
        0.to_spvalue(), // buffer empty
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let product_kinds = &[
        100.to_spvalue(), // unknown type
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let poison = m.add_estimated_bool("poison");

    let product_1_kind = m.add_product_domain("product_1_kind", product_kinds);
    let product_2_kind = m.add_product_domain("product_2_kind", product_kinds);
    let product_3_kind = m.add_product_domain("product_3_kind", product_kinds);

    let shelf1 = m.add_product_domain("shelf1", buffer_domain);
    let shelf2 = m.add_product_domain("shelf2", buffer_domain);
    let shelf3 = m.add_product_domain("shelf3", buffer_domain);
    let conveyor = m.add_product_domain("conveyor", buffer_domain);
    let dorna_holding = m.add_product_domain("dorna_holding", buffer_domain);
    let dorna2_holding = m.add_product_domain("dorna2_holding", buffer_domain);
    let conveyor2 = m.add_product_domain("conveyor2", buffer_domain);

    let dorna_moving = m.get_resource(&r1).get_predicate("moving");
    let dorna2_moving = m.get_resource(&r2).get_predicate("moving");

    let ap = m.get_resource(&r1).get_variable("act_pos");
    let rp = m.get_resource(&r1).get_variable("ref_pos");
    let pp = m.get_resource(&r1).get_variable("prev_pos");

    let ap3 = m.get_resource(&r2).get_variable("act_pos");
    let rp3 = m.get_resource(&r2).get_variable("ref_pos");
    let pp3 = m.get_resource(&r2).get_variable("prev_pos");

    let blue = m.get_resource(&control_box).get_variable("blue_light_on");

    let cf = m.get_resource(&camera).get_predicate("finished");

    let camera_start = m.get_resource(&camera).take_transition("start");

    let cr = m.get_resource(&camera).get_variable("result");
    let cd = m.get_resource(&camera).get_variable("do_scan");

    let gripper_part = m.get_resource(&gripper).get_variable("has_part");
    let gripper_closed = m.get_resource(&gripper).get_variable("is_closed");
    let gripper_opening = m.get_resource(&gripper).get_variable("open");
    let gripper_closing = m.get_resource(&gripper).get_variable("close");
    let gripper_fc = m.get_resource(&gripper).get_variable("fail_count");

    // define robot movement

    // pre_take can be reached from all positions.

    // shelves can be reached from each other and pre_take

    m.add_invar(
        "to_take1",
        &p!([p:ap == t1] => [[p:pp == t1] || [p:pp == t2] || [p:pp == t3] || [p:pp == pt]]),
    );
    m.add_invar(
        "to_take2",
        &p!([p:ap == t2] => [[p:pp == t1] || [p:pp == t2] || [p:pp == t3] || [p:pp == pt]]),
    );
    m.add_invar(
        "to_take3",
        &p!([p:ap == t3] => [[p:pp == t1] || [p:pp == t2] || [p:pp == t3] || [p:pp == pt]]),
    );

    // if we are holding a part, we cannot open the gripper freely!
    m.add_invar(
        "gripper_open",
        &p!([[p:gripper_opening] && [p: dorna_holding !=0]] =>
            [[[p:ap == t1] && [p:shelf1 == 0]] ||
             [[p:ap == t2] && [p:shelf2 == 0]] ||
             [[p:ap == t3] && [p:shelf3 == 0]] ||
             [[p:ap == leave] && [p:conveyor == 0]]]),
    );

    // we can only close the gripper
    m.add_invar(
        "gripper_close",
        &p!([p:gripper_closing] => [[p:ap == t1] || [p:ap == t2] || [p:ap == t3] || [p:ap == leave]]),
    );

    m.add_invar("close_gripper_while_dorna_moving", &p!(![[p:gripper_closing] && [p:dorna_moving]]));
    m.add_invar("open_gripper_while_dorna_moving", &p!(![[p:gripper_opening] && [p:dorna_moving]]));

    // dont open gripper again after failure unless we have moved away.
    m.add_invar("dont_open_gripper_after_failure",
                &p!([[p:gripper_opening] && [[p:ap == t1] || [p:ap == t2] || [p:ap == t3] || [p:ap == leave]]] => [p:gripper_part]));

    // if the gripper is closed, with no part in it, it is impossible for it to hold a part.
    // m.add_invar("gripper_no_sensor_implies_no_part",
    //             &p!([[p:gripper_closed] && [!p:gripper_part]] => [ p:dorna_holding == 0 ]));

    // if there is something on the shelves, we can only try to move there with the gripper open.
    m.add_invar(
        "to_take1_occupied",
        &p!([[p:rp == t1] && [p: dorna_moving]] => [[p:shelf1 == 0] || [! p:gripper_closed]]),
    );

    m.add_invar(
        "to_take2_occupied",
        &p!([[p:rp == t2] && [p: dorna_moving]] => [[p:shelf2 == 0] || [! p:gripper_closed]]),
    );

    m.add_invar(
        "to_take3_occupied",
        &p!([[p:rp == t3] && [p: dorna_moving]] => [[p:shelf3 == 0] || [! p:gripper_closed]]),
    );

    m.add_invar(
        "to_conveyor_occupied",
        &p!([[p:rp == leave] && [p: dorna_moving]] => [[p:conveyor == 0] || [! p:gripper_closed]]),
    );

    // scan and leave can only be reached from pre_take
    m.add_invar(
        "to_scan",
        &p!([p:ap == scan] => [[p:pp == scan] || [p:pp == pt]]),
    );
    m.add_invar(
        "to_leave",
        &p!([p:ap == leave] => [[p:pp == leave] || [p:pp == pt]]),
    );

    // we must always be blue when going to scan
    m.add_invar("blue_scan", &p!([p:rp == scan] => [p:blue]));
    // but only then...
    m.add_invar(
        "blue_scan_2",
        &p!([[p:rp == t1]||[p:rp == t2]||[p:rp == t3]||[p:rp == leave]] => [!p:blue]),
    );

    // we can only scan the product in front of the camera
    let synced_camera_start = camera_start.synchronize("scan_with_dorna_in_place",
                                                       p!([p:ap == scan] && [!p:dorna_moving]), &[]);
    m.add_transition(synced_camera_start);

    // dorna take/leave products
    let pos = vec![
        (t1, shelf1.clone()),
        (t2, shelf2.clone()),
        (t3, shelf3.clone()),
        (leave, conveyor.clone()),
    ];

    let extra = p!([p: product_1_kind <-> p: product_1_kind] &&
                   [p: product_2_kind <-> p: product_2_kind] &&
                   [p: product_3_kind <-> p: product_3_kind]);

    for (pos_name, pos) in pos.iter() {
        m.add_op(&format!("r1_take_{}", pos.leaf()),
                 // operation model guard.
                 &Predicate::AND(vec![p!([p: pos != 0] && [p: dorna_holding == 0]), extra.clone()]),
                 // operation model effects.
                 &[a!(p:dorna_holding <- p:pos), a!(p: pos = 0)],
                 // low level goal
                 &p!([p: ap == pos_name] && [p: gripper_part]),
                 // low level actions (should not be needed)
                 &[],
                 // auto
                 true, Some(p!([p: gripper_fc == 0] || [p: gripper_fc == 1])));

        let goal = if pos_name == &t1 {
            p!([p: ap == pos_name] && [! p: gripper_part] && [!p: poison])
        } else {
            p!([p: ap == pos_name] && [! p: gripper_part])
        };

        m.add_op(&format!("r1_leave_{}", pos.leaf()),
                 // operation model guard.
                 &Predicate::AND(vec![p!([p: dorna_holding != 0] && [p: pos == 0]), extra.clone()]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna_holding), a!(p: dorna_holding = 0)],
                 // low level goal
                 &goal,
                 // low level actions (should not be needed)
                 &[],
                 // auto
                 true, Some(p!([p: gripper_fc == 0] || [p: gripper_fc == 1])));
    }

    // dorna2 take/leave products
    let pos = vec![
        (leave, conveyor.clone()),
        (pt, conveyor2.clone()),
    ];

    for (pos_name, pos) in pos.iter() {
        let buffer_predicate = if pos_name == &pt {
            // when taking the product, because we have an auto transition that consumes it,
            // we need to be sure that the product will still be there
            p!([p: dorna2_holding == 0] && [
                [[p: pos == 1] && [p: product_1_kind == 100]] ||
                    [[p: pos == 2] && [p: product_2_kind == 100]] ||
                    [[p: pos == 3] && [p: product_3_kind == 100]]
            ])
        } else {
            p!([p: pos != 0] && [p: dorna2_holding == 0])
        };

        m.add_op(&format!("r3_take_{}", pos.leaf()),
                 // operation model guard.
                 &buffer_predicate,
                 // operation model effects.
                 &[a!(p:dorna2_holding <- p:pos), a!(p: pos = 0)],
                 // low level goal
                 &p!([p: ap3 == pos_name] && [!p: dorna2_moving]),
                 // low level actions (should not be needed)
                 &[],
                 // not auto
                 false, None);

        m.add_op(&format!("r3_leave_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: dorna2_holding != 0] && [p: pos == 0]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna2_holding), a!(p: dorna2_holding = 0)],
                 // low level goal
                 &p!([p: ap3 == pos_name] && [!p: dorna2_moving]),
                 // low level actions (should not be needed)
                 &[],
                 // not auto
                 false, None);
    }

    let np = |p: i32| {
        p!([p: shelf1 != p]
           && [p: shelf2 != p]
           && [p: shelf3 != p]
           && [p: dorna_holding != p]
           && [p: dorna2_holding != p]
           && [p: conveyor != p]
           && [p: conveyor2 != p]
        )
    };

    let products = &[(1, product_1_kind.clone()),
                     (2, product_2_kind.clone()),
                     (3, product_3_kind.clone())];

    for p in products {
        let n = p.0;

        // scan to figure out the which product we are holding
        let kind = p.1.clone();

        m.add_op_alt(&format!("scan_{}", n),
                     &p!([p: dorna_holding == n] && [p: kind == 100]),
                     &[
                         (&[a!(p: kind = 1)], &p!([p: cf] && [p: cr == 1] && [p: ap == scan]), &[a!(!p: cd)]),
                         (&[a!(p: kind = 2)], &p!([p: cf] && [p: cr == 2] && [p: ap == scan]), &[a!(!p: cd)]),
                         (&[a!(p: kind = 3)], &p!([p: cf] && [p: cr == 3] && [p: ap == scan]), &[a!(!p: cd)]),
                     ],
                     true, None);

        // product sink is at conveyor2, only accepts identified products.
        m.add_op(&format!("consume_known_product_{}", n),
                 // operation model guard.
                 &p!([p: conveyor2 == n] && [p: kind != 100]),
                 // operation model effects.
                 &[a!(p: conveyor2 = 0), a!(p: kind = 100)],
                 // low level goal
                 &Predicate::TRUE,
                 // low level actions (should not be needed)
                 &[],
                 // auto
                 true, None);

        // product source also at conveyor2, we can add a new, unknown,
        // unique product if there is room.
        m.add_op(&format!("add_conveyor2_{}", n),
                 // operation model guard.n
                 &Predicate::AND(vec![p!([p: conveyor2 == 0] && [p: dorna2_holding == 0]), np(p.0)]),
                 // operation model effects.
                 &[a!(p:conveyor2 = n), a!(p: kind = 100)],
                 // low level goal: away from buffer and not moving. OR the robot is not holding anything.
                 &p!([[[p:ap3 != pt] && [p: ap3 <-> p: rp3]] || [p: dorna2_holding == 0]]),
                 //&p!([p:ap3 != pt] && [p: ap3 <-> p: rp3]),
                 // low level actions (should not be needed)
                 &[],
                 // not auto
                 false, None);
    }


    // HIGH LEVEL OPS

    let no_products = p!([p: shelf1 == 0]
           && [p: shelf2 == 0]
           && [p: shelf3 == 0]
           && [p: dorna_holding == 0]
           && [p: dorna2_holding == 0]
           && [p: conveyor == 0]
           && [p: conveyor2 == 0]
        );

    m.add_intention(
        "identify_and_consume_parts",
        false,
        //&p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3]),
        &Predicate::FALSE,
        &no_products,
        &[
            // a!(p: shelf1 = 1),
            // a!(p: shelf2 = 2),
            // a!(p: shelf3 = 3),
            // a!(p: product_1_kind = 100),
            // a!(p: product_2_kind = 100),
            // a!(p: product_3_kind = 100),
        ],
    );

    // m.add_intention(
    //     "get_new_products",
    //     false,
    //     &Predicate::FALSE,
    //     &p!([p: shelf1 == 1] && [p: shelf2 == 0] && [p: shelf3 == 0] &&
    //         [p: product_1_kind == 100]),
    //     &[],
    //     None,
    // );

    m.add_intention(
        "get_new_products",
        false,
        &Predicate::FALSE,
        &p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3] &&
            [p: product_1_kind == 100] && [p: product_2_kind == 100] &&
            [p: product_3_kind == 100]),
        &[],
    );

    m.add_intention(
        "identify_types_r_g_b",
        false,
        &Predicate::FALSE,
        &p!([[p: shelf1 != 0] && [p: shelf2 != 0] && [p: shelf3 != 0] && [
            [[p: product_1_kind == 1] && [p: product_2_kind == 2] && [p: product_3_kind == 3]] ||
             [[p: product_1_kind == 2] && [p: product_2_kind == 3] && [p: product_3_kind == 1]] ||
             [[p: product_1_kind == 3] && [p: product_2_kind == 1] && [p: product_3_kind == 2]] ||
             [[p: product_1_kind == 1] && [p: product_2_kind == 3] && [p: product_3_kind == 2]] ||
             [[p: product_1_kind == 2] && [p: product_2_kind == 1] && [p: product_3_kind == 3]] ||
             [[p: product_1_kind == 3] && [p: product_2_kind == 2] && [p: product_3_kind == 1]]]
        ]
        ),
        &[],
    );

    m.add_intention(
        "identify_two_blue",
        false,
        &Predicate::FALSE,
        &p!([[[p: product_1_kind == 3] && [p: product_2_kind == 3]] &&
             [[[p: shelf1 == 1] && [p: shelf2 == 2]] || [[p: shelf2 == 1] && [p: shelf3 == 2]] || [[p: shelf1 == 1] && [p: shelf3 == 2]]]] ||
            [[[p: product_2_kind == 3] && [p: product_3_kind == 3]] &&
             [[[p: shelf1 == 2] && [p: shelf2 == 3]] || [[p: shelf2 == 2] && [p: shelf3 == 3]] || [[p: shelf1 == 2] && [p: shelf3 == 3]]]] ||
            [[[p: product_1_kind == 3] && [p: product_3_kind == 3]] &&
             [[[p: shelf1 == 1] && [p: shelf2 == 3]] || [[p: shelf2 == 1] && [p: shelf3 == 3]] || [[p: shelf1 == 1] && [p: shelf3 == 3]]]]),
        &[],
    );

    m.add_intention(
        "sort_shelves_r_g_b",
        false,
        &Predicate::FALSE,
        &p!([[[p: product_1_kind == 1] => [p: shelf1 == 1]] &&
             [[p: product_1_kind == 2] => [p: shelf2 == 1]] &&
             [[p: product_1_kind == 3] => [p: shelf3 == 1]] &&
             [[p: product_2_kind == 1] => [p: shelf1 == 2]] &&
             [[p: product_2_kind == 2] => [p: shelf2 == 2]] &&
             [[p: product_2_kind == 3] => [p: shelf3 == 2]] &&
             [[p: product_3_kind == 1] => [p: shelf1 == 3]] &&
             [[p: product_3_kind == 2] => [p: shelf2 == 3]] &&
             [[p: product_3_kind == 3] => [p: shelf3 == 3]] &&
             [[p: product_1_kind != 100] && [p: product_2_kind != 100] && [p: product_3_kind != 100]]
        ]),
        // &p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3] &&
        //     [p: product_1_kind == 1] && [p: product_2_kind == 2] && [p: product_3_kind == 3]
        // ),
        &[],
    );

    // ensure uniqueness of products
    let vars = vec![&shelf1, &shelf2, &shelf3, &conveyor,
                    &dorna_holding, &dorna2_holding, &conveyor2];
    for v in &vars {
        let v = v.clone();
        let ne = Predicate::AND(vars.iter().filter(|&&o|o!=v).map(|&o| p!(p:v <!> p:o)).collect());
        m.add_product_invar(&format!("unique_{}", v.leaf()), &p!([p: v != 0] => [pp: ne]));
    }

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    let initial_state = SPState::new_from_values(&[
        (pp, pt.to_spvalue()), // TODO: move to measured in robot driver?
        (pp3, pt.to_spvalue()),
        (dorna_holding, 0.to_spvalue()),
        (dorna2_holding, 0.to_spvalue()),
        (shelf1, 0.to_spvalue()), //SPValue::Unknown),
        (shelf2, 0.to_spvalue()),
        (shelf3, 0.to_spvalue()),
        (product_1_kind, 100.to_spvalue()), // unknown products
        (product_2_kind, 100.to_spvalue()),
        (product_3_kind, 100.to_spvalue()),
        (conveyor, 0.to_spvalue()),
        (conveyor2, 0.to_spvalue()),
        (poison, false.to_spvalue()),
        (gripper_fc, 0.to_spvalue()),
    ]);

    // operations start in init
    let op_state = m
        .operations
        .iter()
        .map(|o| (o.path().clone(), "i".to_spvalue()))
        .collect::<Vec<_>>();

    // intentions are initially "paused"
    let intention_state = m
        .intentions
        .iter()
        .map(|i| (i.path().clone(), "paused".to_spvalue()))
        .collect::<Vec<_>>();

    let mut s = SPState::new_from_values(op_state.as_slice());
    s.extend(SPState::new_from_values(intention_state.as_slice()));

    // intention state can be set manually in the initial state
    s.extend(initial_state);

    let resource_init_state: SPState = 
        m
        .resources
        .iter()
        .map(|r| {
            r.initial_state()
        })
        .fold(SPState::new(), |mut state, r_state| {
            state.extend(r_state);
            state
        }
    );

    s.extend(resource_init_state);
    
    return (m, s);
}


