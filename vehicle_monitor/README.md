# Mav Saver

The Vehicle Monitor Library (VML) is a C++ library designed to monitor the behavior of a set of flying machines. Its main objective is to process input data representing the state of a set of vehicles and comparing their poses and projected movement according to a given representation of the space they can move in. Depending on these data, the Vehicle Monitor Library checks if a given set of constraints are satisfied and provides as output the result of these checks. 

##Configuration Parameters

The Vehicle Monitor is configured by means of the following inputs:
*	A bounding volume (a box) that roughly wraps the environment in which the flying machines can move.
*	An OctoMap  that defines the portions of the environment that are considered not safe to fly (i.e. obstacles).
*	A list of flying machine, where for each vehicle the Vehicle Monitor must know the identifier and the bounding sphere that wraps the vehicle.
 
##Run-time Inputs

At runtime the Vehicle Monitor has to be feed with the following inputs:
*	The 6D pose of the flying machines, which is generally an information coming from a motion-capture system.
*	The state of a possible emergency button (pressed / not pressed).

##Run-time checks

The Vehicle Monitor can be configured to check a list of constraints. This is done every time new information about the pose of the vehicles or about the state of the emergency button is provided as input. For each constraint that is not satisfied, the Vehicle Monitor produces as output a flag and the last pose in which the vehicle satisfied the constraint. 

The constraints are verified by constraint checkers, which are plugins of the library. The users are allowed to define the list of constraints that have to be checked by choosing from a set of constraint checkers already available and/or by implementing new ones. Some of the available constraint checkers can be configured by means of parameters and/or plugins used to better estimate the state of the vehicle (see for example the velocity estimator described below).

The constraint checkers that are currently available are described in the following list:

* Out of boundaries
 * Constraint description: this constraint requires the vehicle to be inside the bounding volume.
 * Parameters: none. 
 * Required plugin: none.
 * Possible flags: *OUT_OF_BOUNDARIES*.
*	Collision avoidance
 * Constraint description: this constraint requires the vehicle to move in such a way that it will not collide. The constraint checker projects the current pose of the vehicle in the future (based on the estimated velocity and the parameter *ProjectionWindow*) and verifies whether its distance to the closest obstacle (described in the OctoMap) will be smaller then *MinAllowedDistance*.
 * Parameters: 
  * *ProjectionWindow*: number of time steps used to compute the future pose of the vehicle. A time step is considered to be the period with which the motion-capture system provides new pose information. 
  * *MinAllowedDistance*: this parameter is an integer that defines the minimum distance between a vehicle and the closest obstacle. The distance in meters can be computed by multiplying this parameter by the radius of the vehicle’s bounding sphere.
 * Required plugin: velocity estimator (see below).
 * Possible flags: *COLLISION*.
* Velocity constraint
 * Constraint description: this constraint requires the vehicle to move with a linear and angular velocities smaller than *MaxLinearVel* and *MaxAngularVel*, respectively.
 * Parameters:
  * *MaxLinearVel*: this parameter is a triplet that defines the maximum allowed linear velocity.
  * *MaxAngularVel*: this parameter is a triplet that defines the maximum allowed angular velocity.
 * Required plugin: velocity estimator (see below).
 * Possible flags: *UNALLOWED_VELOCITY*.
*	Emergency button
 * Constraint description: this constraint requires the emergency button to be not pressed.
 * Parameters: none. 
 * Required plugin: none.
 * Possible flags: *EMERGENCY_BUTTON*. 

The velocity estimator is a plugin used to run the Velocity Constraint and the Collision Avoidance checkers. It defines an interface to estimate the velocity of a flying machine. In the current implementation the velocity is computed as the derivative of the vehicle pose with respect to the time (where the delta time is computed by dividing the frame number of the pose provided by the motion-capture system by the motion-capture system frequency). The plugin is designed in such a way that it can be easily replaced by a better estimator implemented by the user (e.g. Kalman Filter).



All collision checking is based on an *octomap* see (http://octomap.github.io/)



