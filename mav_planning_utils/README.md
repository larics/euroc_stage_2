# mav_planning_utils

Vertices are support points for the polynomials. Each vertex has a set of constraints:
usually the position only for vertices along the path, while start and end vertices usually have all derivatives of position set to zero.
Vertices are connected by segments, in our case polynomials.
```
  x----------x-----------------x
vertex            segment
```

## How to generate polynomial segments trough a set of waypoints (linear method)

Necessary includes:
```c++
#include <mav_planning_utils/polynomial_optimization.h>
```

First, we create a list of three vertices to fly from 0, 0, 1 over 1, 2, 3 to 2, 1, 5

```c++
mav_planning_utils::Vertex::Vector vertices;
mav_planning_utils::Vertex start(3), middle(3), end(3);
```
The vector argument for the vertices denotes the spatial dimension.

Now, add constraints to the vertices:
```c++
Eigen::Vector3d start_pos(0, 0, 1);
start.makeStartOrEnd(start_pos, mav_planning_utils::derivative_order::SNAP);
//start.addConstraint(mav_planning_utils::derivative_order::VELOCITY, Eigen::Vector3d(1,1,0)); // useful, if there is a starting condition != 0
vertices.push_back(start);

middle.addConstraint(mav_planning_utils::derivative_order::POSITION, Eigen::Vector3d(1, 2, 3));
vertices.push_back(middle);

end.makeStartOrEnd(Eigen::Vector3d(2, 1, 5), mav_planning_utils::derivative_order::SNAP);
vertices.push_back(end);
```

Now, all constraints are defined. The linear optimizer requires the segment-times:
```c++
std::vector<double> segment_times;
double v_max = 2;
double a_max = 2;
double magic_fabian_constant = 6.5; // tuning parameter ... 6.5 is default.
segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);
```

Create an optimizer object and solve. The template parameter (N) denotes the number of coefficients of the underlying polynomial, which has to be even.
If we want the trajectories to be snap-continuous, N needs to be 10 at least. The derivative to optimize should usually be set to the last derivative that should be continuous, hence snap in our case.
```c++
const int N = 10;
const int dimension = 3; // 3D path.
const int derivative_to_optimize = mav_planning_utils::derivative_order::SNAP;
mav_planning_utils::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();
```

Obtain polynomial segments:
```c++
mav_planning_utils::Segment<N>::Vector segments;
opt.getSegments(&segments);
```

## Non-linear optimization:
TODO

## Publishing segments as ros-message:
```c++
#include <mav_planning_utils/ros_trajectory_interface.h>

planning_msgs::PolynomialTrajectory4DPtr trajectory(new planning_msgs::PolynomialTrajectory4D);
mav_planning_utils::polynomialSegmentToPolynomialTrajectoryMsg<N>(segments, &(*trajectory));
```
There are various functions for conversions also including yaw segments. Have a look at the header here:
https://github.com/ethz-asl/ethzasl_mav_pathplanning/blob/master/mav_planning_utils/include/mav_planning_utils/ros_trajectory_interface.h

## Sampling polynomials

Create a Polynomial Trajectory class from the list of segments:
```c++
#include <mav_planning_utils/polynomial_trajectory.h>

mav_planning_utils::PolynomialTrajectory<N> trajectory(dimension, segments);

// single sample:
double sampling_time = 2.0;
Eigen::VectorXd sample = trajectory.evaluate(sampling_time, mav_planning_utils::derivative_order::POSITION);

// sample range:
double t_start = 2.0;
double duration = 10.0;
double dt = 0.01;
std::vector<Eigen::VectorXd> result;
std::vector<double> sampling_times; // optional
trajectory.evaluate_range(t_start, duration, dt, mav_planning_utils::derivative_order::POSITION, &result, &sampling_times);
```

Alternatively, there are convenience functions, that sample the trajectory and convert samples to ```mav_planning_utils::FlatMavState``` or to a vector of it:
```c++
#include <mav_planning_utils/trajectory_sampling.h>

double sampling_time = 2.0;
double sampling_interval = 0.01;
mav_planning_utils::FlatMavState flat_state;
mav_planning_utils::FlatMavState::Vector flat_states;

// single sample:
bool success = sampleTrajectory(const trajectory_position, trajectory_yaw, sample_time, &flat_state);

// sample whole trajectory:
success = sampleWholeTrajectory(trajectory_position, trajectory_yaw, sampling_interval, &flat_states);
```
There are overloads available, where yaw can be omitted. In that case yaw is considered 0.

## Drawing polynomials

### Create visualization_msgs::MarkerArray

```c++
#include <mav_planning_utils/ros_trajectory_interface.h>

visualization_msgs::MarkerArray markers;
bool success = drawMavTrajectory(position_trajectory, yaw_trajectory, &markers);
```
If a vector of flat states exists already:
```c++
bool success = drawMavTrajectory(flat_states, &markers);
```

In case we want to render additional markers, such as a hex-rotor:
```c++
double approx_distance_between_markers = 1.0; //meters
mav_viz::HexacopterMarker hex;

bool success = drawMavTrajectory(position_trajectory, yaw_trajectory, hex, approx_distance_between_markers, &markers);

// or

bool success = drawMavTrajectory(flat_states, hex, approx_distance_between_markers, &markers);
```
In all cases, there exist versions without yaw, as above.

By default, the marker Properties are set as follows:
 - stamp: ros::Time::now()
 - seq: not set
 - frame_id: world
 - marker.action: Marker::ADD
 - marker.lifetime: 0 (i.e. infinite)
 - marker.id: 0 ... n_markers-1

This can be changed by using
```c++
void setMarkerProperties(const std_msgs::Header& header, double life_time, const visualization_msgs::Marker::_action_type& action, visualization_msgs::MarkerArray* markers);
```


### Publish path periodically
```c++
ros::Publisher publisher;
ros::NodeHandle nh;
publisher = nh.advertise<visualization_msgs::MarkerArray>("path", 1);

double publishing_period = 5.0; // seconds

mav_planning_utils::MarkerPublisher marker_publisher(publisher, publishing_period);

// call this whenever your markers get updated
marker_publisher.updateMarkers(markers);
```


