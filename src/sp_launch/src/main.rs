use sp_domain::*;
use sp_runner::*;

#[tokio::main]
async fn main(){
    launch_model(Model::new("empty"), SPState::new()).await.unwrap();
}
