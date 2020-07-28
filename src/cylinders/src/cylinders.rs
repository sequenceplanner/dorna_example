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

    let ce = camera.find_item("enabled", &[]);
    let cf = camera.find_item("finished", &[]);
    let cs = camera.find_item("started", &[]);
    let cr = &camera["result"];

    let gripper_part = &gripper["part_sensor"];
    let gripper_closed = &gripper["closed"];

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
    m.add_invar("product_at_camera", &p!([p:cs] => [p:ap == scan]));

    // dorna take/leave products
    let pos = vec![
        (t1, shelf1.clone()),
        (t2, shelf2.clone()),
        (t3, shelf3.clone()),
        (leave, conveyor.clone()),
    ];

    for (pos_name, pos) in pos.iter() {
        m.add_op(&format!("take_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: pos != 0] && [p: dorna_holding == 0]),
                 // operation model effects.
                 &[a!(p:dorna_holding <- p:pos), a!(p: pos = 0)],
                 // low level goals and actions
                 &[
                     // move in with gripper open
                     (p!([p: ap == pos_name] && [!p: gripper_closed]), &[]),
                     // make sure we get a part
                     (p!([p: ap == pos_name] && [p: gripper_part]), &[])
                 ],
                 // resets
                 true);

        m.add_op(&format!("leave_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: dorna_holding != 0] && [p: pos == 0]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna_holding), a!(p: dorna_holding = 0)],
                 // low level goals and actions
                 &[
                     // move to position with the part
                     (p!([p: ap == pos_name] && [p: gripper_part]), &[]),
                     // release it
                     (p!([p: ap == pos_name] && [!p: gripper_part]), &[]),
                 ],
                 // resets
                 true);
    }

    // scan to figure out which product we are holding
    m.add_op_alt("scan",
                 // operation model guard.
                 &p!([p: dorna_holding == 100]),
                 // operation model (alternative) effects.
                 &[("1", &[a!(p: dorna_holding = 1)]),
                   ("2", &[a!(p: dorna_holding = 2)]),
                   ("3", &[a!(p: dorna_holding = 3)])],
                 // low level goals and actions
                 &[
                     // first enable the camera to clear out any old result
                     (p!(p: ce), &[]),
                     // on camera finish => copy camera result into dorna_holding
                     (p!(p: cf), &[a!(p: dorna_holding <- p: cr)]),
                 ],
                 // resets
                 true);

    // product sink is at conveyor, only accepts identified products.
    m.add_op("consume_known_product",
             // operation model guard.
             &p!([p: conveyor != 0] && [p: conveyor != 100]),
             // operation model effects.
             &[a!(p: conveyor = 0)],
             // low level goals and actions (there are none for this operation)
             &[],
             // resets
             true);



    // Some SOP-testing
    let my_goal = m.add_estimated_bool("my_goal", true);
    m.add_op("the_sop",
        &p!(p: my_goal == false),
        &[a!(p: my_goal = true)],
        &[],
        true,
    );
    let sop_path = m.find("state", &["the_sop", "operations"]);
    
    // m.add_auto_op("step1", &p!(p: sop_path == "e"), &[], &[(p!(p: ap==scan),&[])], false);
    // let step1_path = m.find("state", &["step1", "operations"]);
    // m.add_auto_op("step2", &p!([p: sop_path == "e"] && [p: step1_path == "f"]), &[], &[(p!(p: ap==leave),&[])], false);
    // let step2_path = m.find("state", &["step2", "operations"]);
    // m.add_auto_op("step3", &p!([p: sop_path == "e"] && [p: step2_path == "f"]), &[], &[(p!(p: ap==scan),&[a!(p: my_goal = true)])], false);




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
        (&my_goal, false.to_spvalue()),
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
