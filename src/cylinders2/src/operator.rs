use sp_domain::*;
use sp_runner::*;
use std::collections::HashMap; // todo: macro depends on this...

pub fn make_operator(name: &str, goals: &[&str]) -> Resource {
    // TODO: Handle multiple operators on the ROS_side
    // domain is a list of saved poses. add "unknown" to this list.
    let mut domain = vec!["unknown"];
    domain.extend(goals.iter());
    resource! {
        name: "sp_operator",
        command!{
            topic: "goal",
            msg_type: "sp_operator_msgs/msg/Goal",

            to_do : domain,
        },
        measured!{
            topic: "state",
            msg_type: "sp_operator_msgs/msg/State",

            doing : domain,
            e_stop: bool,
        },
        estimated!{
            prev_pos: domain,
        },

        ability!{
            name: move_to,

            enabled : p!(to_do <-> doing),
            executing : p!(to_do <!> doing),
            finished : p!(to_do <-> doing),

            *start : p!(enabled) => [ a!(prev_pos <- doing), a!(to_do?) ] / [a!(doing = "unknown")],
            finish : p!(executing) => [] / [a!(doing <- to_do)],
        },

        never!{
            name: cannot_go_to_unknown,
            prop: p!(to_do == "unknown")
        },

    }
}
