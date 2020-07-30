use r2r;
use r2r::std_msgs;
use failure::Error;
use std::cell::RefCell;
use std::rc::Rc;

use std::sync::mpsc;
use std::thread;

use opcua_client::prelude::*;


fn main() -> Result<(), Error> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "opc_ros2", "opc")?;
    
    println!("all {:?}", node.params);
    println!("name {:?}", node.name().unwrap());
    
    let res: Vec<(&String, &r2r::ParameterValue)>  = node.params.iter().filter(|&(k, v)| {
        k.contains("opc_variables")
    }).collect();
    
    println!("{:?}", res);
    
    let publisher = node.create_publisher::<std_msgs::msg::String>("read")?;
    
    let (tx, rx) = mpsc::channel::<String>();
    thread::spawn(move || loop {
        let msg = rx.recv().unwrap();
        let deserialized: std_msgs::msg::String = serde_json::from_str(&msg).unwrap();  
        println!(
            "received: {}, deserialized ros msg = {:?}",
            msg, deserialized
        );
    });
    
    let name = node.name().unwrap();
    thread::spawn(move || loop {
        opc(&name, "opc.tcp://localhost:4855/");
    });

    let cb = move |x: std_msgs::msg::String| {
        let serialized = serde_json::to_string(&x).unwrap();
        tx.send(serialized).unwrap(); // pass msg on to other thread for printing
        // let to_send = std_msgs::msg::String { data: to_send };
        // publisher.publish(&to_send).unwrap();
    };

    let _ws2 = node.subscribe("write", Box::new(cb))?;


    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }

}

fn opc(name: &str, server: &str) -> Result<u32, StatusCode> {
    let mut client = ClientBuilder::new()
        .application_name(name)
        .application_uri("urn:".to_string()+name)
        .create_sample_keypair(true)
        .trust_server_certs(true)
        .session_retry_limit(3)
        .client().unwrap();

        
    let endpoint: EndpointDescription = (server, "None", MessageSecurityMode::None, UserTokenPolicy::anonymous()).into();

    // Create the session
    let session = client.connect_to_endpoint(endpoint, IdentityToken::Anonymous).unwrap();

    {
        let mut session = session.write().unwrap();
        let subscription_id = session.create_subscription(2000.0, 10, 30, 0, 0, true, DataChangeCallback::new(|changed_monitored_items| {
            println!("Data change from server:");
            changed_monitored_items.iter().for_each(|item| println!("{:?}", item));
        }))?;
        
        // Create some monitored items
        let items_to_create: Vec<MonitoredItemCreateRequest> = ["v1", "v2", "v3", "v4"].iter()
            .map(|v| NodeId::new(2, *v).into()).collect();
        let _ = session.create_monitored_items(subscription_id, TimestampsToReturn::Both, &items_to_create);
    }

    let _ = Session::run(session);
    Ok(0)
    
}
