import os
import rospy
import rospkg
import rosservice

from functools import partial

from asctec_hl_comm.msg import mav_status

from mav_msgs.msg import Status

from std_msgs.msg import String
from sensor_fusion_comm.msg import ExtEkf
from sensor_fusion_comm.msg import DoubleArrayStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget, QIcon

from .qwt_data_plot import QwtDataPlot
from .qwt_data_plot import LineStyle
from .srv_widget import SrvWidget

from std_srvs.srv import Empty

import tf




# Main widget for the copter GUI
# TODO: Move the subscribers to seperate class?
class CopterPlugin(Plugin):

    namespace = ''

    # Time parameters
    # TODO(millanea): Currently we have two different times for status and state.
    #                 Whether this is required should be rechecked.
    state_plot_start_time = -1
    status_plot_start_time = -1
    status_time = 0
    state_time = 0

    # Control Flags
    pause = 0

    # Status Parameters
    vehicle_name = ''
    vehicle_type = 'unspecified'
    battery_voltage = 0
    flight_time = 0
    system_uptime = ''
    cpu_load = 0.0
    motor_status = ''
    gps_status = ''
    gps_num_satellites = 0
    safety_pilot_status = ''

    # TODO(millanea): The following are not used at the moment. Implement or remove.
    # flight_mode_ll = ''#'flight_mode'
    # state_estimation = ''#'state_estimate'
    # position_control = ''#'position_control'

    # A dictionary specifying the low voltage levels for vehicle type
    kLowVoltageDefault = 10.0
    kLowVoltageNeo = 14.3
    kLowVoltageFirefly = 10.7
    low_votage_threshold_dictionary = { 'unspecified': kLowVoltageDefault,
                                        'neo': kLowVoltageNeo,
                                        'firefly': kLowVoltageFirefly }

    def __init__(self, context):
        super(CopterPlugin, self).__init__(context)
        self.setObjectName('CopterPlugin')

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_copter'), 'resource', 'CopterPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('CopterPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Add icons to the buttons
        self._widget.pause_resume_plot_button_state.setIcon(QIcon.fromTheme('media-playback-pause'))
        self._widget.start_reset_plot_button_state.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.pause_resume_plot_button_controller.setIcon(QIcon.fromTheme('media-playback-pause'))
        self._widget.start_reset_plot_button_controller.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.srv_refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.battery_alert.setIcon(QIcon.fromTheme('dialog-warning'))
        self._widget.battery_alert.setVisible(0)

        # Initialize the timer
        self._start_time = rospy.get_time()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._timer_update)
        self._timer.start(50)

        # Initialize all plots and subscribers
        self._state_subscriber = None
        self._status_subscriber = None
        self._safety_pilot_status_subscriber = None
        self._odometry_subscriber = None
        self._trajectory_subscriber = None
        self._client = None
        self._takeoff_service = None
        self._position_hold_service_name = None
        self._create_plots()
        self._get_init_services()

        # Disabling the unneeded text boxes
        self._widget.position_control_textbox.setDisabled(True)
        self._widget.gps_status_textbox.setDisabled(True)
        self._widget.gps_num_satellites_box.setDisabled(True)
        self._widget.flight_mode_ll_textbox.setDisabled(True)
        self._widget.state_estimation_textbox.setDisabled(True)

        # Add event functions
        self._widget.start_reset_plot_button_state.clicked.connect(self._reset_plots)
        self._widget.pause_resume_plot_button_state.clicked.connect(self._pause_resume_plots)
        self._widget.start_reset_plot_button_controller.clicked.connect(self._reset_plots)
        self._widget.pause_resume_plot_button_controller.clicked.connect(self._pause_resume_plots)
        self._widget.copter_namespace_textbox.returnPressed.connect(self._reset_namespace)
        self._widget.copter_namespace_button.clicked.connect(self._reset_namespace)
        self._widget.srv_refresh_button.clicked.connect(self._refresh_init_services)
        self._widget.copter_namespace_textbox.setFocus()
        # Mav Control Buttons
        self._widget.takeoff_button.clicked.connect(self._takeoff)
        self._widget.position_hold_button.clicked.connect(self._position_hold)

    # Update the displayed data in the widget
    def _timer_update(self):
        # Determininging low voltage threshold from the vehicle type
        if self.vehicle_type in self.low_votage_threshold_dictionary:
            self.lowVoltageThreshold = self.low_votage_threshold_dictionary[self.vehicle_type]
        else:
            self.lowVoltageThreshold = self.low_votage_threshold_dictionary['unspecified']
        # Checking if the voltage is low
        if self.battery_voltage < self.lowVoltageThreshold:
            self._widget.battery_alert.setVisible(1)
        else:
            self._widget.battery_alert.setVisible(0)

        # Updates common to all tabs
        self._widget.vehicle_type_textbox.setText(self.vehicle_type)
        self._widget.battery_voltage_display.setValue(self.battery_voltage)
        self._widget.safety_pilot_status_textbox.setText(self.safety_pilot_status)

        # Only update the current tab
        tab = self._widget.tab_widget.currentIndex()
        # This is the status tab update
        if tab is 0:
            # Updating the status message boxes
            self.plot_battery_voltage.rescale_axis_y()
            self._widget.flight_time_textbox.setText(str(self.flight_time))
            self._widget.cpu_load_bar.setValue(self.cpu_load*100.0) # Converting to percent and displaying
            self._widget.motor_status_textbox.setText(self.motor_status)
            self._widget.gps_status_textbox.setText(self.gps_status)
            self._widget.gps_num_satellites_box.setValue(self.gps_num_satellites)
            # TODO(millanea): The following are not used at the moment. Implement or remove.
            # self._widget.flight_mode_ll_textbox.setText(self.flight_mode_ll)
            # self._widget.state_estimation_textbox.setText(self.state_estimation)
            # self._widget.position_control_textbox.setText(self.position_control)

        # This is the state-plot tab
        if tab is 1 and not self.pause:
            self.plot_position.rescale_axis_y()
            self.plot_velocity.rescale_axis_y()
            self.plot_acceleration_bias.rescale_axis_y()
            self.plot_scale.rescale_axis_y()
        # This is the controller plot tab
        if tab is 2 and not self.pause:
            self.plot_odometry_position.rescale_axis_y()
            self.plot_odometry_orientation.rescale_axis_y()
            self.plot_odometry_velocity.rescale_axis_y()
            self.plot_odometry_rate.rescale_axis_y()

    def _pause_resume_plots(self):
        if self.pause is 0:
            self.pause = 1
            self._widget.pause_resume_plot_button_state.setIcon(QIcon.fromTheme('media-playback-start'))
            self._widget.pause_resume_plot_button_state.setText("Resume")
            self._widget.pause_resume_plot_button_controller.setIcon(QIcon.fromTheme('media-playback-start'))
            self._widget.pause_resume_plot_button_controller.setText("Resume")
        else:
            self.pause = 0
            self._widget.pause_resume_plot_button_state.setIcon(QIcon.fromTheme('media-playback-pause'))
            self._widget.pause_resume_plot_button_state.setText("Pause")
            self._widget.pause_resume_plot_button_controller.setIcon(QIcon.fromTheme('media-playback-pause'))
            self._widget.pause_resume_plot_button_controller.setText("Pause")

    # Get all available msf-initialisation services
    def _get_init_services(self):
        self._init_services = []
        service_names = sorted(rosservice.get_service_list())
        for service_name in service_names:
            if "initialize_msf" in service_name:
                self._init_services.append(SrvWidget(self._widget.srv_container_layout, service_name))

    def _refresh_init_services(self):
        for init_service in self._init_services:
            self._widget.srv_container_layout.removeWidget(init_service._widget)
            init_service._widget.close()
        self._get_init_services()

    # Resetting the namespace
    def _reset_namespace(self):
        self._reset_subscriber()
        self._reset_services_proxies()

    # Subscribe to rostopics according to the namespace
    def _reset_subscriber(self):
        self.namespace = self._widget.copter_namespace_textbox.text()

        state_topic = self.namespace + '/msf_core/state_out'
        status_topic = self.namespace + '/status'
        safety_pilot_status_topic = self.namespace + '/safety_pilot_status'
        odometry_topic = self.namespace + '/msf_core/odometry'
        trajectory_topic = self.namespace + '/command/trajectory'
        print "Subscribing to", state_topic
        print "Subscribing to", status_topic
        print "Subscribing to", safety_pilot_status_topic
        print "Subscribing to", odometry_topic
        print "Subscribing to", trajectory_topic

        if self._state_subscriber is not None:
            self._state_subscriber.unregister()
        if self._status_subscriber is not None:
            self._status_subscriber.unregister()
        if self._safety_pilot_status_subscriber is not None:
            self._safety_pilot_status_subscriber.unregister()
        if self._odometry_subscriber is not None:
            self._odometry_subscriber.unregister()
        if self._trajectory_subscriber is not None:
            self._trajectory_subscriber.unregister()

        self._state_subscriber = rospy.Subscriber(state_topic, DoubleArrayStamped, self._state_subscriber_callback)
        self._status_subscriber = rospy.Subscriber(status_topic, Status, self._status_subscriber_callback)
        self._safety_pilot_status_subscriber = rospy.Subscriber(safety_pilot_status_topic, String, self._safety_pilot_status_subscriber_callback)
        self._odometry_subscriber = rospy.Subscriber(odometry_topic, Odometry, self._odometry_subscriber_callback)
        self._trajectory_subscriber = rospy.Subscriber(trajectory_topic, MultiDOFJointTrajectory, self._trajectory_subscriber_callback)

    def _reset_services_proxies(self):
        self.namespace = self._widget.copter_namespace_textbox.text()

        takeoff_service_name = self.namespace + '/takeoff'
        position_hold_service_name = self.namespace + '/back_to_position_hold'
        print "Creating service proxy for", takeoff_service_name
        print "Creating service proxy for", position_hold_service_name

        try:
            self._takeoff_service = rospy.ServiceProxy(takeoff_service_name, Empty)
        except rospy.ServiceException as exc:
            print "Unable to setup service proxy: " + str(exc)

        try:
            self._position_hold_service = rospy.ServiceProxy(position_hold_service_name, Empty)
        except rospy.ServiceException as exc:
            print "Unable to setup service proxy: " + str(exc)

    def _state_subscriber_callback(self, input):
        # Initializing plot start time on first call
        if self.state_plot_start_time is -1:
            self.state_plot_start_time = input.header.stamp.to_sec()

        # Extracting time of current message
        self.state_time = input.header.stamp.to_sec() - self.state_plot_start_time

        #if self.plot_position is not None:
        self.plot_position.update_value('x', self.state_time, input.data[0])
        self.plot_position.update_value('y', self.state_time, input.data[1])
        self.plot_position.update_value('z', self.state_time, input.data[2])

        #if self.plot_velocity is not None:
        self.plot_velocity.update_value('x', self.state_time, input.data[3])
        self.plot_velocity.update_value('y', self.state_time, input.data[4])
        self.plot_velocity.update_value('z', self.state_time, input.data[5])

        #if self.plot_acceleration_bias is not None:
        self.plot_acceleration_bias.update_value('x', self.state_time, input.data[13])
        self.plot_acceleration_bias.update_value('y', self.state_time, input.data[14])
        self.plot_acceleration_bias.update_value('z', self.state_time, input.data[15])

        #if self.plot_scale is not None:
        self.plot_scale.update_value('scale', self.state_time, input.data[16])

    def _safety_pilot_status_subscriber_callback(self, input):
        self.safety_pilot_status = input.data

    def _status_subscriber_callback(self, input):
        # Initializing plot start time on first call
        if self.status_plot_start_time is -1:
            self.status_plot_start_time = input.header.stamp.to_sec()

        # Extracting time of current message
        self.status_time = input.header.stamp.to_sec() - self.status_plot_start_time

        # Saving contents of the status message
        self.vehicle_name = input.vehicle_name
        self.vehicle_type = input.vehicle_type
        self.battery_voltage = input.battery_voltage        # Battery voltage in V.
        self.flight_time = input.flight_time                # Flight time in s.
        self.system_uptime = input.system_uptime            # MAV uptime in s.
        self.cpu_load = input.cpu_load                      # MAV CPU load: 0.0 ... 1.0
        self.motor_status = input.motor_status              # Current motor status: running, stopped, starting, stopping.
        self.gps_status = input.gps_status                  # GPS status: lock, no_lock
        self.gps_num_satellites = input.gps_num_satellites  # Number of visible satellites

        # TODO(millanea): Used to be used. Reimpliment or remove.
        # self.status_time = input.header.stamp.to_sec() - self.state_plot_start_time
        # self.flight_mode_ll = input.flight_mode_ll
        # self.state_estimation = input.state_estimation
        # self.position_control = input.position_control

        #if self.plot_battery_voltage is not None:
        self.plot_battery_voltage.update_value('voltage', self.status_time, input.battery_voltage)

    def _odometry_subscriber_callback(self, input):
        # Initializing plot start time on first call
        if self.state_plot_start_time is -1:
            self.state_plot_start_time = input.header.stamp.to_sec()

        # Extracting time of current message
        self.state_time = input.header.stamp.to_sec() - self.state_plot_start_time

        self.plot_odometry_position.update_value('x', self.state_time, input.pose.pose.position.x)
        self.plot_odometry_position.update_value('y', self.state_time, input.pose.pose.position.y)
        self.plot_odometry_position.update_value('z', self.state_time, input.pose.pose.position.z)

        orientation_quaternion = (input.pose.pose.orientation.x, input.pose.pose.orientation.y, input.pose.pose.orientation.z, input.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_quaternion, axes='sxyz')
        self.plot_odometry_orientation.update_value('x', self.state_time, roll)
        self.plot_odometry_orientation.update_value('y', self.state_time, pitch)
        self.plot_odometry_orientation.update_value('z', self.state_time, yaw)

        # Rotating velocities in the body frame (from odometry message) to world frame
        # Here we perform rotation through quaternion conjugation
        # R(q)v = q * v_quat * q^(-1)     (Where v_quat is the quaternion representation of the vector v)
        velocity_quaternion = (input.twist.twist.linear.x, input.twist.twist.linear.y, input.twist.twist.linear.z, 0 )
        (velocity_x, velocity_y, velocity_z) = tf.transformations.quaternion_multiply(
                                                    tf.transformations.quaternion_multiply(orientation_quaternion, velocity_quaternion),
                                                    tf.transformations.quaternion_conjugate(orientation_quaternion) )[:3]

        self.plot_odometry_velocity.update_value('x', self.state_time, velocity_x) #self.state_time, input.twist.twist.linear.x)
        self.plot_odometry_velocity.update_value('y', self.state_time, velocity_y) #self.state_time, input.twist.twist.linear.y)
        self.plot_odometry_velocity.update_value('z', self.state_time, velocity_z) #self.state_time, input.twist.twist.linear.z)

        # Rotating roll rates in the body frame (from odometry message) to world frame
        # Here we perform rotation through quaternion conjugation
        # R(q)v = q * v_quat * q^(-1)     (Where v_quat is the quaternion representation of the vector v)
        rate_quaternion = (input.twist.twist.angular.x, input.twist.twist.angular.y, input.twist.twist.angular.z, 0 )
        (rate_x, rate_y, rate_z) = tf.transformations.quaternion_multiply(
                                                    tf.transformations.quaternion_multiply(orientation_quaternion, rate_quaternion),
                                                    tf.transformations.quaternion_conjugate(orientation_quaternion) )[:3]

        self.plot_odometry_rate.update_value('x', self.state_time, rate_x)
        self.plot_odometry_rate.update_value('y', self.state_time, rate_y)
        self.plot_odometry_rate.update_value('z', self.state_time, rate_z)

    def _trajectory_subscriber_callback(self, input):
        # TODO(millanea): Trajectories are not time stamped. Current plotting with last state time. 

        # if self.state_plot_start_time is -1:
        #     self.state_plot_start_time = input.header.stamp.to_sec()

        #self.state_time = input.header.stamp.to_sec() - self.state_plot_start_time
        # self.trajectory_time = input.points[0].time_from_start.to_sec() - self.plot_trajectory_start_time

        self.plot_odometry_position.update_value('x_ref', self.state_time, input.points[0].transforms[0].translation.x)
        self.plot_odometry_position.update_value('y_ref', self.state_time, input.points[0].transforms[0].translation.y)
        self.plot_odometry_position.update_value('z_ref', self.state_time, input.points[0].transforms[0].translation.z)

        orientation_quaternion = (input.points[0].transforms[0].rotation.x, input.points[0].transforms[0].rotation.y, input.points[0].transforms[0].rotation.z, input.points[0].transforms[0].rotation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_quaternion, axes='sxyz')
        self.plot_odometry_orientation.update_value('x_ref', self.state_time, roll)
        self.plot_odometry_orientation.update_value('y_ref', self.state_time, pitch)
        self.plot_odometry_orientation.update_value('z_ref', self.state_time, yaw)

        self.plot_odometry_velocity.update_value('x_ref', self.state_time, input.points[0].velocities[0].linear.x)
        self.plot_odometry_velocity.update_value('y_ref', self.state_time, input.points[0].velocities[0].linear.y)
        self.plot_odometry_velocity.update_value('z_ref', self.state_time, input.points[0].velocities[0].linear.z)

        self.plot_odometry_rate.update_value('x_ref', self.state_time, input.points[0].velocities[0].angular.x)
        self.plot_odometry_rate.update_value('y_ref', self.state_time, input.points[0].velocities[0].angular.y)
        self.plot_odometry_rate.update_value('z_ref', self.state_time, input.points[0].velocities[0].angular.z)

    def _create_plots(self):
        self.state_plot_start_time = -1

        self.plot_battery_voltage = QwtDataPlot(self._widget)
        self._widget.plot_battery_voltage_layout.addWidget(self.plot_battery_voltage)
        self.plot_battery_voltage.add_curve('voltage', 'Voltage', [0], [0])

        self.plot_position = QwtDataPlot("Position", self._widget)
        self._widget.plot_position_layout.addWidget(self.plot_position)
        self.plot_position.add_curve('x', 'x', [0], [0])
        self.plot_position.add_curve('y', 'y', [0], [0])
        self.plot_position.add_curve('z', 'z', [0], [0])

        self.plot_velocity = QwtDataPlot("Velocity", self._widget)
        self._widget.plot_velocity_layout.addWidget(self.plot_velocity)
        self.plot_velocity.add_curve('x', 'x', [0], [0])
        self.plot_velocity.add_curve('y', 'y', [0], [0])
        self.plot_velocity.add_curve('z', 'z', [0], [0])

        self.plot_acceleration_bias = QwtDataPlot("Accelerometer Biases", self._widget)
        self._widget.plot_acceleration_bias_layout.addWidget(self.plot_acceleration_bias)
        self.plot_acceleration_bias.add_curve('x', 'x', [0], [0])
        self.plot_acceleration_bias.add_curve('y', 'y', [0], [0])
        self.plot_acceleration_bias.add_curve('z', 'z', [0], [0])

        self.plot_scale = QwtDataPlot("Scale", self._widget)
        self._widget.plot_scale_layout.addWidget(self.plot_scale)
        self.plot_scale.add_curve('scale', 'visual scale', [0], [0])

        # Controller Plots
        self.plot_odometry_position = QwtDataPlot("Real and Commanded Positions", self._widget)
        self._widget.plot_odometry_position_layout.addWidget(self.plot_odometry_position)
        self.plot_odometry_position.add_curve('x', 'x', [0], [0])
        self.plot_odometry_position.add_curve('y', 'y', [0], [0])
        self.plot_odometry_position.add_curve('z', 'z', [0], [0])
        self.plot_odometry_position.add_curve('x_ref', 'x-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_position.add_curve('y_ref', 'y-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_position.add_curve('z_ref', 'z-ref', [0], [0], line_style=LineStyle.Dashed)

        self.plot_odometry_orientation = QwtDataPlot("Real and Commanded Orientations", self._widget)
        self._widget.plot_odometry_orientation_layout.addWidget(self.plot_odometry_orientation)
        #self.plot_odometry_orientation.add_curve('w', 'w', [0], [0])
        self.plot_odometry_orientation.add_curve('x', 'x', [0], [0])
        self.plot_odometry_orientation.add_curve('y', 'y', [0], [0])
        self.plot_odometry_orientation.add_curve('z', 'z', [0], [0])
        #self.plot_odometry_orientation.add_curve('w_ref', 'w-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_orientation.add_curve('x_ref', 'x-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_orientation.add_curve('y_ref', 'y-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_orientation.add_curve('z_ref', 'z-ref', [0], [0], line_style=LineStyle.Dashed)

        self.plot_odometry_velocity = QwtDataPlot("Real and Commanded Velocities", self._widget)
        self._widget.plot_odometry_velocity_layout.addWidget(self.plot_odometry_velocity)
        self.plot_odometry_velocity.add_curve('x', 'x', [0], [0])
        self.plot_odometry_velocity.add_curve('y', 'y', [0], [0])
        self.plot_odometry_velocity.add_curve('z', 'z', [0], [0])
        self.plot_odometry_velocity.add_curve('x_ref', 'x-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_velocity.add_curve('y_ref', 'y-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_velocity.add_curve('z_ref', 'z-ref', [0], [0], line_style=LineStyle.Dashed)

        self.plot_odometry_rate = QwtDataPlot("Real and Commanded Roll-Rates", self._widget)
        self._widget.plot_odometry_rate_layout.addWidget(self.plot_odometry_rate)
        self.plot_odometry_rate.add_curve('x', 'x', [0], [0])
        self.plot_odometry_rate.add_curve('y', 'y', [0], [0])
        self.plot_odometry_rate.add_curve('z', 'z', [0], [0])
        self.plot_odometry_rate.add_curve('x_ref', 'x-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_rate.add_curve('y_ref', 'y-ref', [0], [0], line_style=LineStyle.Dashed)
        self.plot_odometry_rate.add_curve('z_ref', 'z-ref', [0], [0], line_style=LineStyle.Dashed)

    def _reset_plots(self):
        self._widget.plot_battery_voltage_layout.removeWidget(self.plot_battery_voltage)
        self.plot_battery_voltage.close()

        self._widget.plot_position_layout.removeWidget(self.plot_position)
        self.plot_position.close()

        self._widget.plot_velocity_layout.removeWidget(self.plot_velocity)
        self.plot_velocity.close()

        self._widget.plot_acceleration_bias_layout.removeWidget(self.plot_acceleration_bias)
        self.plot_acceleration_bias.close()

        self._widget.plot_scale_layout.removeWidget(self.plot_scale)
        self.plot_scale.close()

        self._widget.plot_odometry_position_layout.removeWidget(self.plot_odometry_position)
        self.plot_odometry_position.close()

        self._widget.plot_odometry_orientation_layout.removeWidget(self.plot_odometry_orientation)
        self.plot_odometry_orientation.close()

        self._widget.plot_odometry_velocity_layout.removeWidget(self.plot_odometry_velocity)
        self.plot_odometry_velocity.close()

        self._widget.plot_odometry_rate_layout.removeWidget(self.plot_odometry_rate)
        self.plot_odometry_rate.close()

        self._create_plots()

    def shutdown_plugin(self):
        self._timer.stop()
        if self._state_subscriber is not None:
            self._state_subscriber.unregister()
        if self._status_subscriber is not None:
            self._status_subscriber.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('namespace', self.namespace)

    def restore_settings(self, plugin_settings, instance_settings):
        self.namespace = instance_settings.value('namespace', '')
        self._widget.copter_namespace_textbox.setText(self.namespace)
        self._reset_subscriber()
        self._reset_services_proxies()

    def _takeoff(self):
        try:
            self._takeoff_service()
        except rospy.ServiceException as exc:
            print "Service not called: " + str(exc)

    def _position_hold(self):
        try:
            self._position_hold_service()
        except rospy.ServiceException as exc:
            print "Service not called: " + str(exc)