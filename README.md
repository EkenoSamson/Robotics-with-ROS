## Applied-robotics-f2024 
# Pick-and-Place

This describes the pick-and-place process implemented in the `highlevel_controller` ROS package. The process uses a sequence of `ROS`actions defined in a `YAML` configuration file to control `gen3` arm and gripper for picking up a cube and placing it in a cup.

## Gripper Positions

The gripper's position is controlled within a range of 0.0 to 0.8, where:

*   **0.0:** Represents the *fully open* position of the gripper.
*   **0.8:** Represents the *fully closed* position of the gripper.

In this implementation, the gripper opens to **0.1** (almost fully open) and closes to **0.61** (a partial closure sufficient for grasping the cube).

## End-Effector Targets and Action Sequence

The pick-and-place process consists of the following sequence of actions, targeting specific end-effector poses and gripper states:

1.  **Starting Position (action\_0):**
   *   **Type:** `pose_action`
   *   **Translation:** \[0.315, 0.10, 0.5]
   *   **Orientation:** \[0.0, 0.0, 0.0] (Euler angles: roll, pitch, yaw)
   *   **Duration:** 5.0 seconds
   *   **Description:** The robot arm moves to its initial starting pose.

2.  **Open Gripper (action\_1):**
   *   **Type:** `gripper_action`
   *   **Position:** 0.1
   *   **Max Effort:** 1.0
   *   **Description:** The gripper opens to a position of 0.1, preparing to grasp the cube.

3.  **Pre-Grasp Position (action\_2):**
   *   **Type:** `pose_action`
   *   **Translation:** \[0.315, 0.10, 0.43]
   *   **Orientation:** \[0.0, 0.0, 0.0]
   *   **Duration:** 3.0 seconds
   *   **Description:** The robot arm moves to a position directly above the cube, preparing to grasp it. Note the z-value is lowered to 0.43.

4.  **Grasp Cube (action\_3):**
   *   **Type:** `gripper_action`
   *   **Position:** 0.61
   *   **Max Effort:** 0.8
   *   **Description:** The gripper closes to a position of 0.61, grasping the cube. The reduced `max_effort` is used to avoid applying excessive force to the cube.

5.  **Move to Cup Position (action\_4):**
   *   **Type:** `pose_action`
   *   **Translation:** \[0.315, 0.0, 0.55]
   *   **Orientation:** \[0.0, 0.0, 0.0]
   *   **Duration:** 6.0 seconds
   *   **Description:** The robot arm moves the grasped cube to a position above the cup.

6.  **Move to Cup Position 2 (action\_5):**
   *   **Type:** `pose_action`
   *   **Translation:** \[0.43, 0.11, 0.6]
   *   **Orientation:** \[0.0, 0.0, 0.0]
   *   **Duration:** 5.0 seconds
   *   **Description:** The robot arm moves the grasped cube to a final position above the cup.

7.  **Release Cube (action\_6):**
   *   **Type:** `gripper_action`
   *   **Position:** 0.1
   *   **Max Effort:** 1.0
   *   **Description:** The gripper opens, releasing the cube into the cup.

## Results

The pick-and-place process generally achieves successful grasps and placements in the simulated environment. However, due to the inherent limitations of Gazebo's physics engine and the use of a dynamic controller, occasional failures can occur. These failures are typically related to:

*   **Grasping inconsistencies:** Gazebo's contact modeling can sometimes lead to the cube slipping or not being grasped correctly.
*   **Dynamic controller behavior:** The dynamic controller, while providing realistic motion, can sometimes exhibit slight oscillations or imprecision, affecting placement accuracy.

Due to these variations, experiments are to be run several times.

## Demos

The following is a list of demos for pick and place simulations.
+ [pick and place demo 1](https://youtu.be/VlHF7LFSPko)
