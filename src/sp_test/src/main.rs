use sp_domain::*;
use sp_model::*;
use sp_runner::*;


#[derive(Resource)]
struct Dorna {
    #[Variable(type = "String", initial = "unknown", domain = "unknown pre_take leave")]
    #[Output(mapping = "ref_pos")]
    pub ref_pos: Variable,

    #[Variable(type = "String", initial = "unknown", domain = "unknown pre_take leave")]
    #[Input(mapping = "act_pos")]
    pub act_pos: Variable,

    #[Variable(type = "String", initial = "unknown", domain = "unknown pre_take leave")]
    pub prev_pos: Variable,
}

impl Dorna {
    pub fn create_transitions(&self, mb: &mut ModelBuilder) {
        mb.add_runner_transition("goto_leave".into(),
                                 p!([self.ref_pos == self.act_pos] && [self.act_pos != "leave"]),
                                 vec![ a!( self.prev_pos = self.act_pos), a!(self.ref_pos = "leave")]);

        mb.add_runner_transition("goto_pre_take".into(),
                                 p!([self.ref_pos == self.act_pos] && [self.act_pos != "pre_take"]),
                                 vec![ a!( self.prev_pos = self.act_pos), a!(self.ref_pos = "pre_take")]);
    }
}

#[derive(Resource)]
struct Model {
    #[Resource]
    pub d1: Dorna,

    #[Resource]
    pub d2: Dorna,
}

#[tokio::main]
async fn main() {

    let m = Model::new("m");
    let mut mb: ModelBuilder = ModelBuilder::from(&m);

    m.d1.create_transitions(&mut mb);
    m.d2.create_transitions(&mut mb);

    mb.add_message(m.d1.setup_outputs("/dorna/r1/goal", "robot_msgs/msg/RobotGoal"));
    mb.add_message(m.d1.setup_inputs("/dorna/r1/measured", "robot_msgs/msg/RobotState"));

    mb.add_message(m.d2.setup_outputs("/dorna/r2/goal", "robot_msgs/msg/RobotGoal"));
    mb.add_message(m.d2.setup_inputs("/dorna/r2/measured", "robot_msgs/msg/RobotState"));


    // Launch and run a short while.
    let rm = RunnerModel::from(mb);
    let r = launch_model(rm);
    let t = tokio::time::timeout(std::time::Duration::from_millis(30000), r).await;
    println!("Timeout: {:?}", t);
}
