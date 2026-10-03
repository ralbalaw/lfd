# Learning from Demonstrations (LfD)

Rashed Albalawi

In progress.

LfD explores learning robot motion preferences from demonstrations and using them in planning and execution.

## Packages

1. `rbt_planning` provides RRT-Connect and TrajOpt planning through OMPL and TrajOpt.
2. `rbt_features` defines features such as orientation and distance to objects.
3. `rbt_irl` implements MaxEnt IRL.
4. `rbt_types` holds shared types.
5. `rlc_executive` uses behavior trees to coordinate planning, trajectory processing, and execution with ROS 2 and MoveIt 2.
6. `rlc_interfaces` defines shared ROS interfaces.
7. `rlc_scene_bridge` connects MoveIt 2 planning scenes to Tesseract, the environment used by TrajOpt.
8. `rlc_utils` provides common conversions and helpers.
9. `rlc_frontend` contains teleoperation and demonstration collection.
10. `rlc_moveit_config` provides the FR3 MoveIt 2 configuration.
11. `rlc_bringup` integrates the FR3 setup with ROS 2 and Gazebo.

## Dependencies

The main dependencies are ROS 2, MoveIt 2, OMPL, Tesseract, TrajOpt, and BehaviorTree.CPP, and FR3 ROS2 package.

The only part not yet in place is `rlc_frontend` for collecting demonstrations.