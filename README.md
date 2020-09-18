Dorna example
====================

Example of Sequence Planner controlling some simulated resources.

Requirements:
-----------------
1. [ROS2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)
2. [Rust](https://rustup.rs/)
3. [NuXmv](https://nuxmv.fbk.eu)
4. llvm and clang(https://rust-lang.github.io/rust-bindgen/requirements.html#clang)
5. [SP ROS messages](https://github.com/sequenceplanner/sp-ros) Download, colcon build and source before building this repo.

Building:
-----------------
```
colcon build
```

To rebuild the generated Rust messages, you can run:
```
colcon build --cmake-args -DCARGO_CLEAN=ON
```

Running:
-----------------

Small example (`sp_model` defaults to `cylinders`):
```
ros2 launch dorna_example simulation.launch.py rviz:=True
```

Slightly larger example:
```
ros2 launch dorna_example simulation.launch.py rviz:=True sp_model:=cylinders2
```
