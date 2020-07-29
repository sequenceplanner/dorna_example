Dorna example
====================

Example of Sequence Planner controlling some simulated resources.

Requirements:
-----------------
1. [ROS2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)
2. [Rust](https://rustup.rs/)
3. [NuXmv](https://nuxmv.fbk.eu)
4. llvm and clang(https://rust-lang.github.io/rust-bindgen/requirements.html#clang)

Building:
-----------------
```
colcon build
```

Running:
-----------------

Small example:
```
ros2 launch dorna_example simulation.launch.py rviz:=True
```

Slightly larger example:
```
ros2 launch dorna_example simulation2.launch.py rviz:=True
```
