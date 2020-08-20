use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_gripper(name: &str) -> Resource {
    resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "gripper_msgs/msg/Goal",

            close : bool,
        },
        measured!{
            topic: "state",
            msg_type: "gripper_msgs/msg/State",

            closed : bool,
            part_sensor : bool,
        },
        estimated!{
            fail_count : vec![0,1,2],
        },

        ability!{
            name: close,

            enabled : p!(!closed),
            executing : p!([close] && [!closed]),
            finished : p!(closed),

            *start0 : p!([enabled] && [fail_count == 0]) => [ a!(close), a!(fail_count = 1) ] / [],
            *start1 : p!([enabled] && [fail_count == 1]) => [ a!(close), a!(fail_count = 2) ] / [],
            finish_part : p!(executing) => [] / [a!(closed), a!(part_sensor)],
            finish_no_part : p!(executing) => [] / [a!(closed), a!(!part_sensor)],
            finished_reset_fc : p!([finished] && [part_sensor] && [fail_count != 0]) => [a!(fail_count = 0)] / [],
        },

        ability!{
            name: open,

            enabled : p!(closed),
            executing : p!([!close] && [closed]),
            finished : p!(!closed),

            *start : p!(enabled) => [ a!(!close) ] / [],
            finish : p!(executing) => [] / [a!(!closed), a!(!part_sensor)],
        },

        never!{
            name: state_does_not_exist,
            prop: p!([!closed] && [part_sensor])
        },

    }
}
