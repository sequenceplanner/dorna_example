use futures::{future, StreamExt};
use opcua_client::prelude::*;
use r2r;
use std::str::FromStr;
use std::sync::{mpsc, Arc, Mutex, RwLock};
use std::collections::HashMap;

use r2r::gripper_msgs;
use r2r::robot_msgs;
use r2r::control_box_msgs;

use serde_json::json;

fn opc_variant_to_serde_value(variant: &Variant) -> serde_json::Value {
    match variant {
        Variant::Empty => serde_json::Value::Null,
        Variant::Boolean(b) => serde_json::Value::Bool(*b),
        Variant::SByte(i8) => serde_json::Value::Number((*i8).into()),
        Variant::Byte(u8) => serde_json::Value::Number((*u8).into()),
        Variant::Int16(i16) => serde_json::Value::Number((*i16).into()),
        Variant::UInt16(u16) => serde_json::Value::Number((*u16).into()),
        Variant::Int32(i32) => serde_json::Value::Number((*i32).into()),
        Variant::UInt32(u32) => serde_json::Value::Number((*u32).into()),
        Variant::Int64(i64) => serde_json::Value::Number((*i64).into()),
        Variant::UInt64(u64) => serde_json::Value::Number((*u64).into()),
        Variant::Float(f32) => serde_json::Value::Number(
            serde_json::Number::from_f64(*f32 as f64).expect("not proper f64"),
        ),
        Variant::Double(f64) => {
            serde_json::Value::Number(serde_json::Number::from_f64(*f64).expect("not proper f64"))
        }
        Variant::String(s) => match s.value() {
            Some(s) => serde_json::Value::String(s.into()),
            None => serde_json::Value::Null,
        },
        _ => panic!("Not implemented"),
    }
}

fn mutate_variant_from_json(variant: &mut Variant, new_value: &serde_json::Value) {
    match variant {
        Variant::Empty => {},
        Variant::Boolean(b) => {
            if !new_value.is_boolean() {
                println!("warning: not a bool");
                return;
            }
            *b = new_value.as_bool().unwrap();
        },
        Variant::SByte(i8) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *i8 = new_value.as_i64().unwrap() as i8;
        },
        Variant::Byte(u8) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *u8 = new_value.as_i64().unwrap() as u8;
        },
        Variant::Int16(i16) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *i16 = new_value.as_i64().unwrap() as i16;
        },
        Variant::UInt16(u16) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *u16 = new_value.as_i64().unwrap() as u16;
        },
        Variant::Int32(i32) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *i32 = new_value.as_i64().unwrap() as i32;
        }
        Variant::UInt32(u32) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *u32 = new_value.as_i64().unwrap() as u32;
        },
        Variant::Int64(i64) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *i64 = new_value.as_i64().unwrap();
        },
        Variant::UInt64(u64) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *u64 = new_value.as_i64().unwrap() as u64;
        },
        Variant::Float(f32) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *f32 = new_value.as_f64().unwrap() as f32;
        },
        Variant::Double(f64) => {
            if !new_value.is_number() {
                println!("warning: not a number");
                return;
            }
            *f64 = new_value.as_f64().unwrap();
        },
        Variant::String(s) => {
            if !new_value.is_string() {
                println!("warning: not a number");
                return;
            }
            *s = new_value.as_str().unwrap().into();
        }
        _ => panic!("Not implemented"),
    }
}

fn setup_opc(
    server: &str,
    node_ids: Vec<String>,
    state: Arc<Mutex<HashMap<String, Variant>>>,
) -> Result<(Arc<RwLock<Session>>, mpsc::Sender<SessionCommand>), StatusCode> {
    let name = "opcua_ros2_bridge";
    let mut client = ClientBuilder::new()
        .application_name(name)
        .application_uri("urn:".to_string() + name)
        .trust_server_certs(true)
        .session_retry_limit(0)
        .session_timeout(1000)
        .client()
        .expect("could not create OPC client");

    let endpoint: EndpointDescription = (
        server,
        "None",
        MessageSecurityMode::None,
        UserTokenPolicy::anonymous(),
    )
        .into();

    // Create the session
    let session = client
        .connect_to_endpoint(endpoint, IdentityToken::Anonymous)?;

    {
        let mut session = session.write().unwrap();
        let state_cb = state.clone();
        let subscription_id = session.create_subscription(
            50.0,
            10,
            30,
            0,
            0,
            true,
            DataChangeCallback::new(move |changed_monitored_items| {
                let mut map = state_cb.lock().unwrap();
                changed_monitored_items.iter().for_each(|item| {
                    match &item.value().value {
                        Some(v) => {
                            *map.entry(item.item_to_monitor().node_id.to_string())
                                .or_insert(v.clone()) = v.clone();
                        },
                        None => {},
                    }
                });
            }),
        )?;

        // Create some monitored items
        let items_to_create: Vec<MonitoredItemCreateRequest> = node_ids
                .iter()
                .map(|v| NodeId::from_str(&*v).expect("invalid node id").into())
                .collect();
        let _ = session.create_monitored_items(
            subscription_id,
            TimestampsToReturn::Both,
            &items_to_create,
        );
    }

    let cmd_sender = Session::run_async(session.clone());
    Ok((session, cmd_sender))
}

fn write_opc(session: Arc<RwLock<Session>>, state: Arc<Mutex<HashMap<String, Variant>>>,
             to_write: serde_json::Value) -> bool {
    // assume to_write is a json object
    let obj = to_write.as_object().expect("command must be a json object");

    let items_to_write = obj.iter().flat_map(|(key, json_value)| {
        // println!("to write: {} - {}", key, json_value);

        // we need to lookup the correct datatype from the current state.
        let state = state.lock().unwrap();
        match state.get(key) {
            Some(v) => {
                let mut new_variant = v.clone();
                mutate_variant_from_json(&mut new_variant, json_value);

                if &new_variant == v {
                    // no change, dont write.
                    None
                } else {
                    // write it.
                    let wv = WriteValue {
                        node_id: NodeId::from_str(key).unwrap(),
                        attribute_id: 13, // Value attribute id
                        index_range: UAString::default(),
                        value: DataValue::value_only(new_variant),
                    };

                    Some(wv)
                }
            },
            None => {
                println!("WARNING: cannot write to {} we havent read the item yet", key);
                None
            }
        }
    }).collect::<Vec<WriteValue>>();

    if items_to_write.is_empty() {
        return true;
    }

    let result = session.write().unwrap().write(&items_to_write);
    match result {
        Ok(r) => {
            r.iter().for_each(|r| {
                r.iter().for_each(|r| {
                    if r.name() != "Good" {
                        println!("WARNING: write result: {} {}", r.name(), r.description());
                    }
                });
            })
        }
        Err(e) => {
            println!("WARNING: write not successful {}", e);
            return false;
        }
    }

    return true;
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "opcua_ros2_bridge", "")?;

    let server_address = if let Some(r2r::ParameterValue::String(addr))
        = node.params.lock().unwrap().get("server_address") {
            addr.clone()
    } else {
        "opc.tcp://localhost:4840/".to_string()
    };
    let node_ids = match node.params.lock().unwrap().get("node_ids") {
        Some(r2r::ParameterValue::StringArray(s)) => s.clone(),
        _ => vec![
            // our hardcoded list.
            "s=s1.pos",
            "s=s2.gripper_closed",
            "s=s3.gripper_part",
            "s=s4.conveyor_sensor",
            "s=s5.conveyor_running",
            "s=a1.pos",
            "s=a2.gripper",
            "s=a3.conv_left",
            "s=do_reset",
            "s=is_reset",
        ].into_iter().map(|s|s.to_string()).collect(),
    };
    println!("listening to node ids: {:?}", node_ids);

    let state = Arc::new(Mutex::new(HashMap::new()));
    let (session, _kill) = setup_opc(&server_address, node_ids, state.clone()).expect("could not connect to opc server");

    let gripper_sub = node.subscribe::<gripper_msgs::msg::Measured>("/r1_gripper/measured")?;
    let gripper_pub = node.create_publisher::<gripper_msgs::msg::Goal>("/r1_gripper/goal")?;

    let robot_sub = node.subscribe::<robot_msgs::msg::RobotState>("/dorna/r1/measured")?;
    let robot_pub = node.create_publisher::<robot_msgs::msg::RobotGoal>("/dorna/r1/goal")?;

    let control_box_sub = node.subscribe::<control_box_msgs::msg::Measured>("/control_box/measured")?;
    let control_box_pub = node.create_publisher::<control_box_msgs::msg::Goal>("/control_box/goal")?;

    tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    let state_task = state.clone();
    let session_task = session.clone();
    tokio::spawn(async {
        gripper_sub.for_each(move |msg| {
            let session = session_task.clone();
            let state_task = state_task.clone();
            let json_object = json!({
                "s=s2.gripper_closed": msg.closed,
                "s=s3.gripper_part": msg.part_sensor,
            });
            write_opc(session, state_task, json_object);
            future::ready(())
        }).await;
    });

    let state_task = state.clone();
    let session_task = session.clone();
    tokio::spawn(async {
        robot_sub.for_each(move |msg| {
            let session = session_task.clone();
            let state_task = state_task.clone();
            let json_object = json!({
                "s=s1.pos": msg.act_pos,
            });
            write_opc(session, state_task, json_object);
            future::ready(())
        }).await;
    });

    let state_task = state.clone();
    let session_task = session.clone();
    tokio::spawn(async {
        control_box_sub.for_each(move |msg| {
            let session = session_task.clone();
            let state_task = state_task.clone();
            let json_object = json!({
                "s=s4.conveyor_sensor": msg.conv_sensor,
                "s=s5.conveyor_running": msg.conv_running_left,

                "s=is_reset": msg.conv_running_right, //hack
            });
            write_opc(session, state_task, json_object);
            future::ready(())
        }).await;
    });

    let state_task = state.clone();
    loop {
        let json_map = {
            let state = state_task.lock().unwrap();
            let mut json_map = serde_json::Map::with_capacity(state.len());
            for (k,v) in &*state {
                json_map.insert(k.clone(), opc_variant_to_serde_value(&v));
            }
            json_map
        };

        // create gripper goal msg based on opc state
        if let Some(close) = json_map.get("s=a2.gripper") {
            let close = close.as_bool().expect("wrong datatype");
            let msg = gripper_msgs::msg::Goal { close };
            gripper_pub.publish(&msg).expect("could not publish");
        }

        // create robot goal msg based on opc state
        if let Some(pos) = json_map.get("s=a1.pos") {
            let pos = pos.as_str().expect("wrong datatype");
            let msg = robot_msgs::msg::RobotGoal { ref_pos: pos.to_string() };
            robot_pub.publish(&msg).expect("could not publish");
        }

        // create control box goal msg based on opc state
        if let Some(conv) = json_map.get("s=a3.conv_left") {
            if let Some(do_reset) = json_map.get("s=do_reset") {
                let conv = conv.as_bool().expect("wrong datatype");
                let do_reset = do_reset.as_bool().expect("wrong datatype");
                let msg = control_box_msgs::msg::Goal { conv_left: conv,
                                                        conv_right: do_reset, ..
                                                        control_box_msgs::msg::Goal::default()};
                control_box_pub.publish(&msg).expect("could not publish");
            }
        }

        // sleep a little.
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;
    }
}
