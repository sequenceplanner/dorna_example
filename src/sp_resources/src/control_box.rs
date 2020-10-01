use sp_domain::*;
use sp_runner::*;

pub fn create_instance(name: &str) -> Resource {
    let mut r = resource! {
        name: name,
        command!{
            topic: "goal",
            msg_type: "control_box_msgs/msg/Goal",

            blue_light : bool,
        },
        measured!{
            topic: "measured",
            msg_type: "control_box_msgs/msg/Measured",

            blue_light_on : bool,
        },

        transitions!{
            c_blue_on_start : p!(!blue_light_on), vec![ a!(blue_light) ],
            e_blue_on_finish : p!([blue_light] && [!blue_light_on]), vec![a!(blue_light_on)],

            c_blue_off_start : p!(blue_light_on), vec![ a!(!blue_light) ],
            e_blue_off_finish : p!([!blue_light] && [blue_light_on]), vec![a!(!blue_light_on)],
        },
    };

    // let mv = vec!(
    //     MessageVariable{name: SPPath::from_string("blue_light_on"), path: SPPath::from_string("measured/0/blue_light_on")},
    // );
    // let mess = NewMessage{
    //     topic: SPPath::from_string("/testing/measured"),
    //     relative_topic: false,
    //     category: MessageCategory::OutGoing,
    //     message_type: MessageType::Ros("control_box_msgs/msg/Measured".to_string()),
    //     variables: mv
    // };

    // r.new_messages = vec!(mess);

    r
}

#[cfg(test)]
mod test {
    use super::*;
    use serial_test::serial;

    #[test]
    #[serial]
    fn test_control_box() {
        let control_box = create_instance("cb");
        println!("{:#?}", control_box);

        let bl = control_box
            .find_item("blue_light", &["cb"])
            .expect("check spelling")
            .path();
        println!("bl: {}", bl);

        let bl_on = control_box
            .find_item("blue_light_on", &["cb"])
            .expect("check spelling")
            .path();
        println!("bl on: {}", bl_on);
    }
}
