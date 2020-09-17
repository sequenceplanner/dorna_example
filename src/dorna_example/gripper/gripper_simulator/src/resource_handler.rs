use r2r;
use std::sync::{Arc, Mutex};
use failure::Error;

use r2r::sp_messages::msg::{RegisterResource, Resources};


pub struct ResourceHandler {
    last: Arc<Mutex<String>>,
}

impl ResourceHandler {
    pub fn new(node: &mut r2r::Node, resource_name: String, initial_goal: String) -> Result<Self, Error> {
        let register_publisher = node.create_publisher::<RegisterResource>("/sp/resource")?;
        let logger = node.logger().to_string();
        let last = Arc::new(Mutex::new(initial_goal));
        let last_cb = Arc::clone(&last);
        let sp_resources_cb = move |msg: Resources| {
            if !msg.resources.contains(&resource_name) {
                let last_goal_from_sp = last_cb.lock().unwrap().clone();
                let register = RegisterResource{
                    path: resource_name.clone(),
                    model: "".to_string(),
                    last_goal_from_sp
                };
                r2r::log_info!(&logger, "The resource {} is not registered in: {:?}. Sending {:?}", &resource_name, msg.resources, register.clone());
                register_publisher.publish(&register).expect(&format!("The register resources publisher has died in node {}", &resource_name));
            }
        };
        let _node_sub = node.subscribe("/sp/resources", Box::new(sp_resources_cb))?;
        Ok(ResourceHandler {last})
    }

    pub fn last_goal(&self, json: String) {
        let mut l = self.last.lock().unwrap();
        *l = json;
    }

    pub fn has_last_goal(&self) -> bool {
        !self.last.lock().unwrap().is_empty()
    }
}
