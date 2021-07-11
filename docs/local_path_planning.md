## local path planning
This robot has a local path planner like DWA (Dynamic Window Approach)

#### (1) find a search area (Dynamic window)
A search area (the maximum and minimum control inputs) is calculated based on the current linear speed and angular velocity and the maximum and minimum velocity and maximum acceleration of the robot.  
This search area is called Dynamic Window.  
The default accelerations or velocities are defined in each Robot like below:

* [`models::robot::MAX_LIN_ACC`](../src/models/robot.rs#L8)
* [`models::robot::MAX_ANG_ACC`](../src/models/robot.rs#L10)
* [`models::robot::MAX_V`](../src/models/robot.rs#L12)
* [`models::robot::MIN_V`](../src/models/robot.rs#L14)
* [`models::robot::MAX_OMEGA`](../src/models/robot.rs#L16)
* [`models::robot::MIN_OMEGA`](../src/models/robot.rs#L18)

But you can override them by your Agent:

* [`agent::waypoints_agent::WaypointsAgent.get_max_accelarations()`](../src/agent/waypoints_agent.rs#L75)
* [`agent::waypoints_agent::WaypointsAgent.get_linear_velocities()`](../src/agent/waypoints_agent.rs#L87)
* [`agent::waypoints_agent::WaypointsAgent.get_angular_velocities()`](../src/agent/waypoints_agent.rs#L99)

> In order to avoid obstacles, you have to eliminate the observed obstacle area from the calculated Dynamic Window above. But this robot does not implement this obstacle avoidance feature.

#### (2) sample the control inputs
The calculated Dynamic Window is sampled at a certain resolution. These sampled values are candidates for the next control input.

* [`planners::dwa_wo_obstacle::V_RESOLUTION`](../src/planners/dwa_wo_obstacle.rs#L13)
* [`planners::dwa_wo_obstacle::OMEG4_RESOLUTION`](../src/planners/dwa_wo_obstacle.rs#L15)

#### (3) evaluate the sampled control inputs
A next robot pose is calculated by using each candidate control input, and each cost value is evaluated by using below cost functions based on the calculated each robot pose.

* [`planners::dwa_wo_obstacle::eval_heading()`](../src/planners/dwa_wo_obstacle.rs#L129)
  * the difference angle between the direction to destination and candidate pose's theta
* [`planners::dwa_wo_obstacle::eval_velocity()`](../src/planners/dwa_wo_obstacle.rs#L141)
  * the difference velocity between max linear velocity and next linear velocity
* [`planners::dwa_wo_obstacle::eval_distance()`](../src/planners/dwa_wo_obstacle.rs#L153)
  * the distance between the destination and candidate pose
* [`planners::dwa_wo_obstacle::eval_theta()`](../src/planners/dwa_wo_obstacle.rs#L165)
  * the difference angle between the destination's theta and candidate pose's theta

The calculated costs above are normalized and multiplied by the weight parameters, and add together.

* When the current position and the target position are far apart:
  * [`planners::dwa_wo_obstacle::FAR_ERROR_ANGLE_GAIN`](../src/planners/dwa_wo_obstacle.rs#L18)
  * [`planners::dwa_wo_obstacle::FAR_VELOCITY_GAIN`](../src/planners/dwa_wo_obstacle.rs#L20)
  * [`planners::dwa_wo_obstacle::FAR_DISTANCE_GAIN`](../src/planners/dwa_wo_obstacle.rs#L22)
  * [`planners::dwa_wo_obstacle::FAR_THETA_GAIN`](../src/planners/dwa_wo_obstacle.rs#L24)
* When the current position and the target position are close:
  * [`planners::dwa_wo_obstacle::NEAR_ERROR_ANGLE_GAIN`](../src/planners/dwa_wo_obstacle.rs#L27)
  * [`planners::dwa_wo_obstacle::NEAR_VELOCITY_GAIN`](../src/planners/dwa_wo_obstacle.rs#L29)
  * [`planners::dwa_wo_obstacle::NEAR_DISTANCE_GAIN`](../src/planners/dwa_wo_obstacle.rs#L31)
  * [`planners::dwa_wo_obstacle::NEAR_THETA_GAIN`](../src/planners/dwa_wo_obstacle.rs#L33)


> In order to avoid obstacles, you have to evaluate a cost of how far from obstacle too. But this robot does not implement this obstacle avoidance feature.

#### (4) select the next control input
The lowest cost candidate is selected as the next control input.

