from __future__ import division

import os
import rospy
import rospkg

from asctec_hl_comm.msg import mav_status
from sensor_fusion_comm.msg import ExtEkf
from sensor_fusion_comm.srv import InitScale
from sensor_fusion_comm.srv import InitHeight

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget, QColor, QLabel, QIcon

from .qwt_data_plot import QwtDataPlot


class CopterPlugin(Plugin):

    plot_start_time = -1
    status_time = 0
    state_time = 0
    pause = 0

    # Observed parameters
    voltage = 10
    cpu_load = 0
    flight_mode_ll = 'flight_mode'
    state_estimation = 'state_estimate'
    position_control = 'position_control'
    motor_status = 'motor_status'
    flight_time = 0
    gps_status = 'gps_status'
    gps_num_satellites = 0

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

        # Add Icon to Allert
        self._widget.battery_alert.setIcon(QIcon.fromTheme('dialog-warning'))
        self._widget.battery_alert.setVisible(0)

        # Initialize the Timer
        self._start_time = rospy.get_time()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._timer_update)
        self._timer.start(50)

        # Initialize all Plots and Subscribers
        self._state_subscriber = None
        self._status_subscriber = None
        self._client = None
        self._create_plots()

        # Add Event Functions
        self._widget.start_reset_plot_button.clicked.connect(self._reset_plots)
        self._widget.pause_resume_plot_button.clicked.connect(self._pause_resume_plots)
        self._widget.copter_namespace_textbox.returnPressed.connect(self._reset_subscriber)
        self._widget.copter_namespace_button.clicked.connect(self._reset_subscriber)

        self._widget.scale_slider.valueChanged.connect(self._scale_slider_change)
        self._widget.scale_spin_box.valueChanged.connect(self._scale_box_change)
        self._widget.height_slider.valueChanged.connect(self._height_slider_change)
        self._widget.height_spin_box.valueChanged.connect(self._height_box_change)

        self._widget.apply_scale_button.clicked.connect(self._init_scale)
        self._widget.apply_height_button.clicked.connect(self._init_height)

        self._widget.copter_namespace_textbox.setFocus()

    def _init_scale(self):
        try:
            initialize_msf_scale = rospy.ServiceProxy('/msf_pose_sensor/pose_sensor/initialize_msf_scale', InitScale)
            res = initialize_msf_scale(self._widget.scale_spin_box.value())
            print res.result
        except rospy.ServiceException:
            print "Service call failed"

    def _init_height(self):
        try:
            initialize_msf_height = rospy.ServiceProxy('/msf_pose_sensor/pose_sensor/initialize_msf_height', InitHeight)
            res = initialize_msf_height(self._widget.height_spin_box.value())
            print res.result
        except rospy.ServiceException:
            print "Service call failed"

    def _timer_update(self):
        # Check if Voltage low
        if self.voltage < 10:
            self._widget.battery_alert.setVisible(1)
        else:
            self._widget.battery_alert.setVisible(0)

        # Update the status tab
        self._widget.cpu_load_bar.setValue(self.cpu_load)
        self._widget.flight_mode_ll_textbox.setText(self.flight_mode_ll)
        self._widget.state_estimation_textbox.setText(self.state_estimation)
        self._widget.position_control_textbox.setText(self.position_control)
        self._widget.motor_status_textbox.setText(self.motor_status)
        self._widget.flight_time_box.setValue(self.flight_time)
        self._widget.gps_status_textbox.setText(self.gps_status)
        self._widget.gps_num_satellites_box.setValue(self.gps_num_satellites)
        self._widget.battery_voltage_display.setValue(self.voltage)

        # Update all Plots if they exist:
        tab = self._widget.tab_widget.currentIndex()
        if tab is 0:
            # this is the status tab
            self.plot_battery_voltage.rescale_axis_y()
        if tab is 1 and not self.pause:
            # this is the state-plot tab
            self.plot_position.rescale_axis_y()
            self.plot_velocity.rescale_axis_y()
            self.plot_acceleration_bias.rescale_axis_y()
            self.plot_scale.rescale_axis_y()

    def _pause_resume_plots(self):
        if self.pause is 0:
            self.pause = 1
            self._widget.pause_resume_plot_button.setText("Resume")
        else:
            self.pause = 0
            self._widget.pause_resume_plot_button.setText("Pause")

    def _reset_subscriber(self):
        # Get the current namespace and the according topics
        namespace = self._widget.copter_namespace_textbox.text()

        state_topic = namespace + '/fcu/ekf_state_out'
        status_topic = namespace + '/fcu/status'
        print "Subscribing to", state_topic
        print "Subscribing to", status_topic

        if self._state_subscriber is not None:
            self._state_subscriber.unregister()
        if self._status_subscriber is not None:
            self._status_subscriber.unregister()

        self._state_subscriber = rospy.Subscriber(state_topic, ExtEkf, self._state_subscriber_callback)
        self._status_subscriber = rospy.Subscriber(status_topic, mav_status, self._status_subscriber_callback)

        # Activate the tab widget at first click
        if not self._widget.tab_widget.isEnabled():
            self._widget.tab_widget.setEnabled(1)

    def _state_subscriber_callback(self, input):
        if self.plot_start_time is -1:
            self.plot_start_time = input.header.stamp.to_sec()

        self.state_time = input.header.stamp.to_sec() - self.plot_start_time

        if self.plot_position is not None:
            self.plot_position.update_value('x', self.state_time, input.state[0])
            self.plot_position.update_value('y', self.state_time, input.state[1])
            self.plot_position.update_value('z', self.state_time, input.state[2])

        if self.plot_velocity is not None:
            self.plot_velocity.update_value('x', self.state_time, input.state[3])
            self.plot_velocity.update_value('y', self.state_time, input.state[4])
            self.plot_velocity.update_value('z', self.state_time, input.state[5])

        if self.plot_acceleration_bias is not None:
            self.plot_acceleration_bias.update_value('x', self.state_time, input.state[6])
            self.plot_acceleration_bias.update_value('y', self.state_time, input.state[7])
            self.plot_acceleration_bias.update_value('z', self.state_time, input.state[8])

        if self.plot_scale is not None:
            self.plot_scale.update_value('scale', self.state_time, input.state[9])

    def _status_subscriber_callback(self, input):
        self.status_time = input.header.stamp.to_sec() - self.plot_start_time
        self.voltage = input.battery_voltage
        self.cpu_load = input.cpu_load
        self.flight_mode_ll = input.flight_mode_ll
        self.state_estimation = input.state_estimation
        self.position_control = input.position_control
        self.motor_status = input.motor_status
        self.flight_time = input.flight_time
        self.gps_status = input.gps_status
        self.gps_num_satellites = input.gps_num_satellites

        if self.plot_battery_voltage is not None:
            self.plot_battery_voltage.update_value('voltage', self.status_time, input.battery_voltage)

    def _create_plots(self):
        self.plot_start_time = -1

        self.plot_battery_voltage = QwtDataPlot(self._widget)
        self._widget.plot_battery_voltage_layout.addWidget(self.plot_battery_voltage)
        self.plot_battery_voltage.add_curve('voltage', 'Voltage', [0], [0])

        self.plot_position = QwtDataPlot(self._widget)
        self._widget.plot_position_layout.addWidget(self.plot_position)
        self.plot_position.add_curve('x', 'x-position', [0], [0])
        self.plot_position.add_curve('y', 'y-position', [0], [0])
        self.plot_position.add_curve('z', 'z-position', [0], [0])

        self.plot_velocity = QwtDataPlot(self._widget)
        self._widget.plot_velocity_layout.addWidget(self.plot_velocity)
        self.plot_velocity.add_curve('x', 'x-velocity', [0], [0])
        self.plot_velocity.add_curve('y', 'y-velocity', [0], [0])
        self.plot_velocity.add_curve('z', 'z-velocity', [0], [0])

        self.plot_acceleration_bias = QwtDataPlot(self._widget)
        self._widget.plot_acceleration_bias_layout.addWidget(self.plot_acceleration_bias)
        self.plot_acceleration_bias.add_curve('x', 'x-acc-bias', [0], [0])
        self.plot_acceleration_bias.add_curve('y', 'y-acc-bias', [0], [0])
        self.plot_acceleration_bias.add_curve('z', 'z-acc-bias', [0], [0])

        self.plot_scale = QwtDataPlot(self._widget)
        self._widget.plot_scale_layout.addWidget(self.plot_scale)
        self.plot_scale.add_curve('scale', 'visual scale', [0], [0])

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

        self._create_plots()

    def _scale_slider_change(self, value):
        self._widget.scale_spin_box.setValue(value/100)

    def _scale_box_change(self, value):
        self._widget.scale_slider.setValue(value*100)

    def _height_slider_change(self, value):
        self._widget.height_spin_box.setValue(value/100)

    def _height_box_change(self, value):
        self._widget.height_slider.setValue(value*100)
    
    def shutdown_plugin(self):
        self._timer.stop()
        if self._state_subscriber is not None:
            self._state_subscriber.unregister()
        if self._status_subscriber is not None:
            self._status_subscriber.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog