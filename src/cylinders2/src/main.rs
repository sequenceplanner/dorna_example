use failure::Error;
use sp_runner::*;

mod camera;
mod control_box;
mod cylinders;
mod dorna;
mod gripper;

fn main() -> Result<(), Error> {
    let (model, initial_state) = cylinders::cylinders();

    launch_model(model, initial_state)?;

    Ok(())
}
