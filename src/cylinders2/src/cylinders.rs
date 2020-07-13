use crate::camera::*;
use crate::control_box::*;
use crate::dorna::*;
use sp_domain::*;
use sp_runner::*;

pub fn cylinders() -> (Model, SPState, Predicate) {
    let mut m = GModel::new("cylinders2");

    let pt = "pre_take";
    let scan = "scan";
    let t1 = "take1"; // shelf poses
    let t2 = "take2";
    let t3 = "take3";
    let leave = "leave"; // down at conveyor

    let dorna = m.use_named_resource("dorna", make_dorna("r1", &[pt, scan, t1, t2, t3, leave]));
    let dorna2 = m.use_named_resource("dorna", make_dorna("r2", &[pt, scan, t1, t2, t3, leave]));
    let dorna3 = m.use_named_resource("dorna", make_dorna("r3", &[pt, scan, leave]));
    let dorna4 = m.use_named_resource("dorna", make_dorna("r4", &[pt, scan, leave]));

    let cb = m.use_resource(make_control_box("control_box"));
    let camera = m.use_resource(make_camera("camera"));

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

    let product_1_kind = m.add_estimated_domain("product_1_kind", product_kinds, true);
    let product_2_kind = m.add_estimated_domain("product_2_kind", product_kinds, true);
    let product_3_kind = m.add_estimated_domain("product_3_kind", product_kinds, true);

    // poison low level only
    let poison = m.add_estimated_bool("poison", false);

    let shelf1 = m.add_estimated_domain("shelf1", buffer_domain, true);
    let shelf2 = m.add_estimated_domain("shelf2", buffer_domain, true);
    let shelf3 = m.add_estimated_domain("shelf3", buffer_domain, true);
    let conveyor = m.add_estimated_domain("conveyor", buffer_domain, true);
    let dorna_holding = m.add_estimated_domain("dorna_holding", buffer_domain, true);
    let dorna3_holding = m.add_estimated_domain("dorna3_holding", buffer_domain, true);
    let conveyor2 = m.add_estimated_domain("conveyor2", buffer_domain, true);

    let x = m.add_estimated_domain("x", &["left".to_spvalue(), "right".to_spvalue()], true);

    let ap = &dorna["act_pos"];
    let rp = &dorna["ref_pos"];
    let pp = &dorna["prev_pos"];
    let blue = &cb["blue_light_on"];

    let cf = camera.find_item("finished", &[]);
    let cs = camera.find_item("started", &[]);
    let cr = &camera["result"];
    let cd = &camera["do_scan"];

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
    // m.add_invar(
    //     "dorna2_1",
    //     &p!([p:ap == scan] => [p:ap2 == leave]),
    // );

    // m.add_invar(
    //     "dorna2_2",
    //     &p!([p:ap == leave] => [p:ap2 == scan]),
    // );


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
    // we need to keep the product still while scanning
    m.add_invar("product_still_at_camera", &p!([p:cd] => [p: ap <-> p: rp]));

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
                 &p!(p: ap == pos_name),
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true);

        let goal = if pos_name == &t1 {
            p!([p: ap == pos_name] && [!p: poison])
        } else {
            p!(p: ap == pos_name)
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
                 // resets
                 true);
    }

    // dorna3 take/leave products
    let pos = vec![
        (leave, conveyor.clone()),
        (pt, conveyor2.clone()),
    ];

    let ap3 = &dorna3["act_pos"];
    let rp3 = &dorna3["ref_pos"];

    for (pos_name, pos) in pos.iter() {
        m.add_op(&format!("r3_take_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: pos != 0] && [p: dorna3_holding == 0]),
                 // operation model effects.
                 &[a!(p:dorna3_holding <- p:pos), a!(p: pos = 0)],
                 // low level goal
                 &p!(p: ap3 == pos_name),
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true);

        m.add_op(&format!("r3_leave_{}", pos.leaf()),
                 // operation model guard.
                 &p!([p: dorna3_holding != 0] && [p: pos == 0]),
                 // operation model effects.
                 &[a!(p:pos <- p:dorna3_holding), a!(p: dorna3_holding = 0)],
                 // low level goal
                 &p!(p: ap3 == pos_name),
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true);
    }


    let ap4 = &dorna4["act_pos"];
    let rp4 = &dorna4["ref_pos"];
    m.add_op("r4_x_left",
             // operation model guard.
             &p!(p: x == "right"),
             // operation model effects.
             &[a!(p:x = "left")],
             // low level goal
             &p!(p: ap4 == leave),
             // low level actions (should not be needed)
             &[],
             // resets
             true);
    m.add_op("r4_x_right",
             // operation model guard.
             &p!(p: x == "left"),
             // operation model effects.
             &[a!(p:x = "right")],
             // low level goal
             &p!(p: ap4 == scan),
             // low level actions (should not be needed)
             &[],
             // resets
             true);

    // force dorna4 to move sometimes by connecting it to dorna 2
    m.add_invar(
        "dorna4_1",
        &p!([p:ap2 == scan] => [p:rp4 == scan]),
    );

    m.add_invar(
        "dorna4_2",
        &p!([p:ap2 == leave] => [p:rp4 == leave]),
    );

    // scan to figure out the which product we are holding

    // this is what we want
    // m.add_spec("take_scan_result1", camera.reset,
    //            &p!([p: dorna_holding == 100] && [p: cr == 1]),
    //            &[a!(p: dorna_holding = 1)]);


    // scan to figure out the which product we are holding
    m.add_op_alt("scan1",
                 // operation model guard.
                 &p!([p: dorna_holding == 1] && [p: product_1_kind == 100]),
                 // operation model (alternative) effects.
                 &[("1", &[a!(p: product_1_kind = 1)]),
                   ("2", &[a!(p: product_1_kind = 2)]),
                   ("3", &[a!(p: product_1_kind = 3)])],
                 // low level goal
                 &p!([p: cf] && [p: cr != 0]),
                 // low level actions (should not be needed)
                 &[a!(p: product_1_kind <- p: cr), a!(!p: cd)], // copy result regardless of outcome and reset camera
                 // &[a!(!p: cd)],                             // only reset the camera. low level planner will scan until cr == 1
                 // &[],                                       // no reset. low level planner will reuse the same result to avoid scanning
                 // resets
                 true);

    // scan to figure out the which product we are holding
    m.add_op_alt("scan2",
                 // operation model guard.
                 &p!([p: dorna_holding == 2] && [p: product_2_kind == 100]),
                 // operation model (alternative) effects.
                 &[("1", &[a!(p: product_2_kind = 1)]),
                   ("2", &[a!(p: product_2_kind = 2)]),
                   ("3", &[a!(p: product_2_kind = 3)])],
                 // low level goal
                 &p!([p: cf] && [p: cr != 0]),
                 // low level actions (should not be needed)
                 &[a!(p: product_2_kind <- p: cr), a!(!p: cd)], // copy result regardless of outcome and reset camera
                 // &[a!(!p: cd)],                             // only reset the camera. low level planner will scan until cr == 1
                 // &[],                                       // no reset. low level planner will reuse the same result to avoid scanning
                 // resets
                 true);

    // scan to figure out the which product we are holding
    m.add_op_alt("scan3",
                 // operation model guard.
                 &p!([p: dorna_holding == 3] && [p: product_3_kind == 100]),
                 // operation model (alternative) effects.
                 &[("1", &[a!(p: product_3_kind = 1)]),
                   ("2", &[a!(p: product_3_kind = 2)]),
                   ("3", &[a!(p: product_3_kind = 3)])],
                 // low level goal
                 &p!([p: cf] && [p: cr != 0]),
                 // low level actions (should not be needed)
                 &[a!(p: product_3_kind <- p: cr), a!(!p: cd)], // copy result regardless of outcome and reset camera
                 // &[a!(!p: cd)],                             // only reset the camera. low level planner will scan until cr == 1
                 // &[],                                       // no reset. low level planner will reuse the same result to avoid scanning
                 // resets
                 true);


    let np = |p: i32| {
        p!([p: shelf1 != p]
           && [p: shelf2 != p]
           && [p: shelf3 != p]
           && [p: dorna_holding != p]
           && [p: dorna3_holding != p]
           && [p: conveyor != p]
           && [p: conveyor2 != p]
        )
    };

    let products = &[(1, product_1_kind.clone()),
                     (2, product_2_kind.clone()),
                     (3, product_3_kind.clone())];

    for p in products {
        // product sink is at conveyor2, only accepts identified products.
        let n = p.0;
        let kind = p.1.clone();
        m.add_op(&format!("consume_known_product_{}", p.0),
                 // operation model guard.
                 &p!([p: conveyor2 == n] && [p: kind != 100]),
                 // operation model effects.
                 &[a!(p: conveyor2 = 0)],
                 // low level goal
                 &Predicate::TRUE,
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true);

        // product source also at conveyor2, we can add a new, unknown,
        // unique product if there is room.
        let kind = p.1.clone();
        m.add_op(&format!("add_conveyor2_{}", p.0),
                 // operation model guard.n
                 &Predicate::AND(vec![p!([p: conveyor2 == 0] && [p: dorna3_holding == 0]), np(p.0)]),
                 // operation model effects.
                 &[a!(p:conveyor2 = p.0), a!(p: kind = 100)],
                 // low level goal: away from buffer and not moving.
                 &p!([p:ap3 != pt] && [p: ap3 <-> p: rp3]),
                 // low level actions (should not be needed)
                 &[],
                 // resets
                 true);
    }


    // HIGH LEVEL OPS

    let no_products = p!([p: shelf1 == 0]
           && [p: shelf2 == 0]
           && [p: shelf3 == 0]
           && [p: dorna_holding == 0]
           && [p: dorna3_holding == 0]
           && [p: conveyor == 0]
           && [p: conveyor2 == 0]
        );




    // m.add_hl_op("identify_parts", true,
    //             &p!([p:shelf1 == 100] && [p:shelf2 == 100] && [p:shelf3 == 100]),
    //             &p!([p:shelf1 == 1] && [p:shelf2 == 2] && [p:shelf3 == 3]),
    //             &[a!(p:shelf1 = 100), a!(p:shelf2 = 100), a!(p:shelf3 = 100)],
    //             None);

    m.add_hl_op(
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
        None,
    );

    m.add_hl_op(
        "get_new_products",
        false,
        &Predicate::FALSE,
        &p!([p: shelf1 == 1] && [p: shelf2 == 2] && [p: shelf3 == 3] &&
            [p: product_1_kind == 100] && [p: product_2_kind == 100] &&
            [p: product_3_kind == 100]),
        &[],
        None,
    );

    m.add_hl_op(
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
        None,
    );

    m.add_hl_op(
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
        None,
    );

    m.add_hl_op(
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
        None,
    );

    m.add_hl_op(
        "to_left",
        true,
        &p!([p: x == "right"]),
        &p!(p: x == "left"),
        &[],
        None,
    );

    m.add_hl_op(
        "to_right",
        true,
        &p!([p: x == "left"]),
        &p!(p: x == "right"),
        &[],
        None,
    );

    // goal for testing
    // let g = p!([p:shelf1 == 1] && [p:shelf2 == 2] && [p:shelf3 == 3]);
    //let g = p!([p:shelf1 == 1]);
    //let g = p!([p: conveyor == 0]);

    let g = p!([p: ap == leave]);

    let pp2 = &dorna2["prev_pos"];
    let pp3 = &dorna3["prev_pos"];
    let pp4 = &dorna4["prev_pos"];

    // setup initial state of our estimated variables.
    // todo: do this interactively in some UI
    m.initial_state(&[
        (pp, pt.to_spvalue()), // TODO: move to measured in robot driver?
        (pp2, pt.to_spvalue()),
        (pp3, pt.to_spvalue()),
        (pp4, leave.to_spvalue()),
        (&dorna_holding, 0.to_spvalue()),
        (&dorna3_holding, 0.to_spvalue()),
        (&shelf1, 0.to_spvalue()), //SPValue::Unknown),
        (&shelf2, 0.to_spvalue()),
        (&shelf3, 0.to_spvalue()),
        (&product_1_kind, 100.to_spvalue()), // unknown products
        (&product_2_kind, 100.to_spvalue()),
        (&product_3_kind, 100.to_spvalue()),
        (&conveyor, 0.to_spvalue()),
        (&conveyor2, 0.to_spvalue()),
        (&x, "left".to_spvalue()),
        (&poison, false.to_spvalue()),
    ]);

    println!("MAKING MODEL");
    let (m, s) = m.make_model();
    (m, s, g)
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    fn compute_resource_use(o: &Operation) {
        println!("{}", o.path());
    }

    #[test]
    #[serial]
    fn operations() {
        let (m, _s, _g) = cylinders();
        let ts_model = TransitionSystemModel::from(&m);

        m
            .all_operations()
            .iter()
            .filter(|o| !o.high_level)
            .for_each(|o| {
                let oo = o.transitions().iter()
                    .find(|t| t.name() == "planning")
                    //.map(|t|t.guard.clone())
                    .expect("planning trans not found");
                println!();
                println!("OP: {}", oo);


                if o.name() == "r1_take_conveyor_1" {

                let g = o.goal().as_ref().map(|g|g.goal.clone()).unwrap();
                let goal = vec![(g, None)];
                let pre = o.transitions().iter()
                    .find(|t| t.name() == "planning")
                    .map(|t|t.guard.clone())
                    .expect("planning trans not found");
                let plan = plan_check(&ts_model, &pre, &goal, 50);
                    println!("PLAN SUCCESS? {}", plan.plan_found);
                }

                // compute_resource_use(o);
                // o.transitions().iter().for_each(|t| println!("{}", t));
            });

        assert!(false);
    }


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
