# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model description

### Model States

The model has the following state variables:
* X position
* Y position
* psi angle (cars absolute angle, not steering angle)
* Speed
* Cross Track Error
* psi Error

For this model X, Y, and psi position of (0,0,0) is considered to be the cars center with the hood of the car pointing along the x axis and the y axis passing left and right through the cars passenger and driver side door.

The speed is measured directly by the simulator and placed into the first state. To calculate the cross track error and psi error, first a 3rd order polynomial is fit to the waypoints the car is told to follow. It is assumed that the waypoints are generated in global coordinate by a different part of the SDC stack. These waypoints are then converted to vehicle coordinates and fit with the polynomial. Based on this generated polynomial, the cross track error is calculated as the point where the polynomial crosses the y axis (x = 0 or not ahead or behind the car). Since the car is at y = 0, this value is the error. The psi error is angle of the line (derivative) at x = 0.


### Actuators

The state control outputs or actuators is the throttle position and steering angle. A 100 ms latency is simulated so that throttle and steering values are not executed 100 ms after they are sent out. However this is handled by MPC by modifying the X, Y states as described below.

### Update equations.

A global kinematic model is used to predict the cars location based on speed and steering angle. These two actuators are able to manipulate all 6 states therefore, in control system terms, the system has full controllability.

The full update equations can be seen in the comments of FG_eval. In order to use the solver, all equations are set equal to zero.

### Latency and preprocessing.

Since the vehicles current position and waypoints are given in global coordinates, the waypoint coordinates are translated and rotated into the vehicles coordinates. If there was no latency, the MPC controller would always receive it's current state of (X,Y) as (0,0).

The cars speed, which is given in MPH, is converted to m/s before it is used by the MPC.

The actuator inputs have a 100 ms delay from being sent the actuation commands and executing them. This means that the MPC controller should predict its inputs not for its current state but for where it will be 100 ms from now based on current steering angle and speed. A simple unicycle model is used to predict the cars X and Y position based on current steering angle and speed. While not perfect, this model is very simple and is accurate for small time steps (100 ms) and small steering angles. Even though X and Y states are now non-zero, psi remains zero as the small steering angle and small time steps assumptions (which minimize the error when using the unicycle model for X and Y positions) do not hold up for the cars angle. Also this value when calculated exactly will still be small so keeping it at zero is a minor assumption that will have minimal impact.

### Cost Function

The cost function is the heart of the MPC as it defines what the controller is trying to minimize by manipulating the actuators. For this project I found that the cost function needed to have a very heavy emphasis on smoothness of the steering input in order for the car to go around the track in a smooth and controlled manner. Thus high cost was placed on change in steering input and total steering input. This way the controller would be encouraged towards smaller and smoother turns than sharp and jerky ones.

For the errors, I found that the error in psi was far more important than cross track error in achieving this. This makes intuitive sense as if error psi is zero, then having a CTE will just mean that the car is taking the corner but either inside or wide of the planned route. Without this large emphasis on epsi over CTE, the car would turn very hard to stay in the center of the lane. This would put the car in situations where it is driving almost directly at the center line of the lane to quickly reduce the CTE. This made for a very jerky and unstable ride.

Since the cost function puts a high cost on steering smoothness and epsi, the car takes almost every turn wide so that it doesn't have to steer as hard as it would on the center line or inside line. While this does make the cars wheel get very close to the outside yellow lines, this ensures the smoothest and most controlled ride possible. If this simulator was on a street as opposed to a race track, the cost function would be changed to put a very high price on CTE to ensure the car stays in its own lane.

### N and dt values.

dt is 0.1 s which is the exact same time as the latency. This is more by coincidence and then by design. I started with the idea of having my prediction horizon be between 1-2 seconds. I started with N of 30 and dt of 0.05 so that I would have 1.5 second prediction horizon. However just visually inspecting the distance between consecutive predictions, even at the cars top speed, seemed as if this level of discretization was overkill. Even though there is no issue in setting dt to a number greater than the latency (as the MPC is still recalculated every 100 ms, regardless of dt), I found that a .1 s dt with 15 time steps to have an acceptable distance (about 3 meters) between consecutive points at the cars max speed.

### Linear Speed Controller

A linear speed controller is used to adjust the cars reference speed (the speed the MPC tries to attain) based on the total cost estimated by the MPC. Thus when approaching a sharp corner with a high cost (due to high CTE or large steering deltas), the reference speed will be decreased. When on a section of the track with a low cost (small difference between predicted path and waypoint path) the reference speed will be increased. A max speed of 65 MPH has been enforced and the car will reach this speed consistently on the back straight.
