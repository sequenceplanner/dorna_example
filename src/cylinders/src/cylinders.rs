use crate::camera::*;
use crate::control_box::*;
use crate::gripper::*;
use crate::dorna::*;
use sp_domain::*;
use sp_runner::*;

pub fn cylinders() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cylinders");

    let pt = "pre_take";
    let scan = "scan";
    let t1 = "take1"; // shelf poses
    let t2 = "take2";
    let t3 = "take3";
    let leave = "leave"; // down at conveyor

    let dorna = m.use_named_resource("dorna", make_dorna("r1", &[pt, scan, t1, t2, t3, leave]));
    let dorna_moving = dorna.find_item("executing", &["move_to"]);
    let dorna2 = m.use_named_resource("dorna", make_dorna("r2", &[pt, scan, t1, t2, t3, leave]));

    let cb = m.use_resource(make_control_box("control_box"));
    let camera = m.use_resource(make_camera("camera"));
    let gripper = m.use_resource(make_gripper("gripper"));

    let product_domain = &[
        100.to_spvalue(), // SPValue::Unknown,   macros need better support for Unknown
        0.to_spvalue(),
        1.to_spvalue(),
        2.to_spvalue(),
        3.to_spvalue(),
    ];

    let shelf1 = m.add_estimated_domain("shelf1", product_domain, true);
    let shelf2 = m.add_estimated_domain("shelf2", product_domain, true);
    let shelf3 = m.add_estimated_domain("shelf3", product_domain, true);
    let conveyor = m.add_estimated_domain("conveyor", product_domain, true);
    let dorna_holding = m.add_estimated_domain("dorna_holding", product_domain, true);

    let ap = &dorna["act_pos"];
    let rp = &dorna["ref_pos"];
    let pp = &dorna["prev_pos"];
    let blue = &cb["blue_light_on"];

    let cf = camera.find_item("finished", &[]);
    let ce = camera.find_item("enabled", &[]);

    let cr = &camera["result"];
    let cd = &camera["do_scan"];

    let gripper_part = &gripper["part_sensor"];
    let gripper_closed = &gripper["closed"];
    let gripper_opening = gripper.find_item("executing", &["open"]);
    let gripper_closing = gripper.find_item("executing", &["close"]);
    let gripper_fc = &gripper["fail_count"];

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
    m.add_invar("dont_open_gripper_after_failure_shelf1",
                &p!([[p:gripper_opening] && [[p:ap == t1] || [p:ap == t2] || [p:ap == t3] || [p:ap == leave]]] => [p:gripper_part]));

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


    // force dorna2 to move sometimes
    let ap2 = &dorna2["act_pos"];
    m.add_invar(
        "dorna2_1",
        &p!([p:ap == scan] => [p:ap2 == leave]),
    );

    m.add_invar(
        "dorna2_2",
        &p!([p:ap == leave] => [p:ap2 == scan]),
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
    m.add_invar("product_at_camera", &p!([!p:ce] => [[p:ap == scan] && [!p:dorna_moving]]));

    // dorna take/leave products
    let pos = vec![
        (t1, shelf1.clone()),
        (t2, shelf2.clone()),
        (t3, shelf3.clone()),
        (leave, conveyor.clone()),
    ];

    for (pos_name, pos) in pos.iter() {

        let buffer_predicate = if pos_name == &leave {
            // when taking the product, because we have an auto transition that consumes it,
            // we need to be sure that the product will still be there
            p!([p: pos == 100] && [p: dorna_holding == 0])
        } else {
            p!([p: pos != 0] && [p: dorna_holding == 0])
        };

        m.add_op(&format!("take_{}", pos.leaf()),
                 // operation model guard.
                 &buffer_predicate,
                 // operation model effects.
                 &[a!(p:dorna_holding <- p:pos), a!(p: pos = 0)],
                 // low level goal
                 &p!([p: ap == pos_name] && [p: gripper_part]),
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true, true, Some(p!([p: gripper_fc == 0] || [p: gripper_fc == 1])));

        m.add_op(&format!("leave_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: dorna_holding != 0] && [p: pos == 0]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna_holding), a!(p: dorna_holding = 0)],
                 // low level goal
                 &p!([p: ap == pos_name] && [! p: gripper_part]),
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true, true, None);
    }


    // this is what we want
    // m.add_spec("take_scan_result1", camera.reset,
    //            &p!([p: dorna_holding == 100] && [p: cr == 1]),
    //            &[a!(p: dorna_holding = 1)]);

    // scan to figure out the which product we are holding
    for i in 1..=3 {
        m.add_op(&format!("scan_{}", i),
                 // operation model guard.
                 &p!([p: dorna_holding == 100]),
                 // operation model (alternative) effects.
                 &[a!(p: dorna_holding = i)],
                 // low level goal
                 &p!([p: cf] && [p: cr == i] && [p: ap == scan]),
                 // low level actions (should not be needed)
                 &[a!(!p: cd)], // reset camera
                 true, true, None);
    }

    // product sink is at conveyor, only accepts identified products.
    m.add_op("consume_known_product",
             // operation model guard.
             &p!([p: conveyor != 0] && [p: conveyor != 100]),
             // operation model effects.
             &[a!(p: conveyor = 0)],
             // low level goal
             &Predicate::TRUE,
             // low level actions (should not be needed)
             &[],
             // resets
             true, true, None);


    // INTENTIONS
    let np = |p: i32| {
        p!([p: shelf1 != p]
            && [p: shelf2 != p]
            && [p: shelf3 != p]
            && [p: dorna_holding != p]
            && [p: conveyor != p])
    };

    let no_products = Predicate::AND(vec![np(1), np(2), np(3), np(100)]);

    m.add_intention(
        "identify_and_consume_parts",
        true,
        &p!([p: shelf1 == 100] && [p: shelf2 == 100] && [p: shelf3 == 100]),
        &no_products,
        &[
            a!(p: shelf1 = 100),
            a!(p: shelf2 = 100),
            a!(p: shelf3 = 100),
        ],
        None,
    );

    // copy of the intention action to low level (testing)....
    m.add_auto("reset_intention", &no_products, &[
        a!(p: shelf1 = 100),
        a!(p: shelf2 = 100),
        a!(p: shelf3 = 100),
    ]);

    // goal for testing
    // let g = p!([p:shelf1 == 1] && [p:shelf2 == 2] && [p:shelf3 == 3]);
    //let g = p!([p:shelf1 == 1]);
    //let g = p!([p: conveyor == 0]);

    let g = p!([p: ap == leave]);

    let pp2 = &dorna2["prev_pos"];

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (pp, pt.to_spvalue()), // TODO: move to measured in robot driver?
        (pp2, scan.to_spvalue()), // TODO: move to measured in robot driver?
        (&dorna_holding, 0.to_spvalue()),
        (&shelf1, 100.to_spvalue()), //SPValue::Unknown),
        (&shelf2, 100.to_spvalue()),
        (&shelf3, 100.to_spvalue()),
        (&conveyor, 0.to_spvalue()),
        (ap, pt.to_spvalue()),
        (rp, pt.to_spvalue()),
        (ap2, pt.to_spvalue()),
        (gripper_fc, 0.to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m, s) = m.make_model();
    (m, s, g)
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_cylinders() {
        let (m, s, g) = cylinders();

        make_runner_model(&m);

        let mut ts_model = TransitionSystemModel::from(&m);

        let mut new_specs = Vec::new();
        for s in &ts_model.specs {
            new_specs.push(Spec::new(s.name(), refine_invariant(&ts_model, s.invariant())));
        }
        ts_model.specs = new_specs;

        let goal = (g, None);
        let plan = plan(&ts_model, &[goal], &s, 50);

        println!("\n\n\n");

        if plan.plan_found {
            plan.trace.iter().enumerate().skip(1).for_each(|(i, t)| {
                println!("{}: {}", i, t.transition);
            });
        } else {
            println!("no plan found");
        }

        println!("\n\n\n");
    }
}
