from __future__ import division

import os
import rospy
import rospkg
import dynamic_reconfigure.client

# TODO get message information correct!
try:
    from asctec_hl_comm.msg import mav_status
except:
    from _mav_status import mav_status
    print "mav_status import failed - using pre-build message"

try:
    from sensor_fusion_comm.msg import ExtEkf
except:
    from _ExtEkf import ExtEkf
    print "ExtEkf import failed - using pre-build message"

# Qt related imports:
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget, QColor, QLabel, QIcon

from .qwt_data_plot import QwtDataPlot


class CopterPlugin(Plugin):

    # 'Buffer' for Plots
    plot_start_time = -1

    voltage = 10
    status_time = 0

    position = [0, 0, 0]
    velocity = [0, 0, 0]
    acceleration_bias = [0, 0, 0]
    scale = 0
    state_time = 0

    pause = 0

    # Observed parameters
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
        # Give QObjects reasonable names
        self.setObjectName('CopterPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_copter'), 'resource', 'CopterPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CopterPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Add Icon to Allert (face-worried, dialog-error, battery-low, dialog-warning)
        self._widget.battery_alert.setIcon(QIcon.fromTheme('dialog-warning'))
        self._widget.battery_alert.setVisible(0)

        # Initialize the Timer
        self._start_time = rospy.get_time()
        # A slow timer for the redrawing and the widget update.
        self._slow_timer = QTimer(self)
        self._slow_timer.timeout.connect(self._slow_timer_update)
        self._slow_timer.start(50)
        # Fast timer for adding plot data for smooth curves
        self._fast_timer = QTimer(self)
        self._fast_timer.timeout.connect(self._fast_timer_update)
        self._fast_timer.start(10)

        # Initialize all Plots and Subscribers
        self.plot_battery_voltage = None
        self.plot_position = None
        self.plot_velocity = None
        self.plot_acceleration_bias = None
        self.plot_scale = None
        self._state_subscriber = None
        self._status_subscriber = None
        self._client = None

        # Add Event Functions
        self._widget.start_reset_plot_button.clicked.connect(self._reset_plots)
        self._widget.pause_resume_plot_button.clicked.connect(self._pause_resume_plots)
        self._widget.copter_namespace_button.clicked.connect(self._reset_subscriber)
        self._widget.connect_dyn_rec.clicked.connect(self._dyn_reconf_connect)

        self._widget.scale_slider.valueChanged.connect(self._scale_slider_change)
        self._widget.scale_spin_box.valueChanged.connect(self._scale_box_change)
        self._widget.height_slider.valueChanged.connect(self._height_slider_change)
        self._widget.height_spin_box.valueChanged.connect(self._height_box_change)

        self._widget.apply_scale_button.clicked.connect(self._apply_scale)
        self._widget.apply_height_button.clicked.connect(self._apply_height)

    def _apply_height(self):
        height = self._widget.height_spin_box.value()
        print "Initializing Filter with height", height
        self._client.update_configuration({"core_height": height})
        self._client.update_configuration({"core_set_height": True})
        self._client.update_configuration({"core_set_height": False})

    def _apply_scale(self):
        scale = self._widget.scale_spin_box.value()
        print "Initializing Filter with scale", scale
        self._client.update_configuration({"pose_initial_scale": scale})
        self._client.update_configuration({"core_init_filter": True})
        self._client.update_configuration({"core_init_filter": False})

    def _dyn_reconf_connect(self):
        services = dynamic_reconfigure.find_reconfigure_services()
        service = None
        for s in services:
            if "msf" in s and not "core" in s:
                service = s

        if service is not None:
            print "Connecting to dynamic_reconfigure service", service, "..."
            try:
                self._client = dynamic_reconfigure.client.Client(service, timeout=5)
                print "Connected"
                self._widget.dyn_rec_frame.setEnabled(1)
            except:
                print "No connection established"
        else:
            print "No suitable service found. Is msf running?"

    def _slow_timer_update(self):
        # Update all Plots if they exist:
        if not self.pause:
            if self.plot_battery_voltage is not None:
                self.plot_battery_voltage.rescale_axis_y()
            if self.plot_position is not None:
                self.plot_position.rescale_axis_y()
            if self.plot_velocity is not None:
                self.plot_velocity.rescale_axis_y()
            if self.plot_acceleration_bias is not None:
                self.plot_acceleration_bias.rescale_axis_y()
            if self.plot_scale is not None:
                   self.plot_scale.rescale_axis_y()

        # Check if Voltage low
        if self.voltage < 10:
            self._widget.battery_alert.setVisible(1)
        else:
            self._widget.battery_alert.setVisible(0)

        # Update the rest
        self._widget.cpu_load_bar.setValue(self.cpu_load)
        self._widget.flight_mode_ll_textbox.setText(self.flight_mode_ll)
        self._widget.state_estimation_textbox.setText(self.state_estimation)
        self._widget.position_control_textbox.setText(self.position_control)
        self._widget.motor_status_textbox.setText(self.motor_status)
        self._widget.flight_time_box.setValue(self.flight_time)
        self._widget.gps_status_textbox.setText(self.gps_status)
        self._widget.gps_num_satellites_box.setValue(self.gps_num_satellites)
        self._widget.battery_voltage_display.setValue(self.voltage)

    def _fast_timer_update(self):
        # This loop is seperate for performance.
        # Plots stay smooth but can be drawn in the slow timer loop in this way.
        if self.plot_battery_voltage is not None:
            self.plot_battery_voltage.update_value('voltage', self.status_time, self.voltage)

        if self.plot_position is not None:
            self.plot_position.update_value('x', self.state_time, self.position[0])
            self.plot_position.update_value('y', self.state_time, self.position[1])
            self.plot_position.update_value('z', self.state_time, self.position[2])

        if self.plot_velocity is not None:
            self.plot_velocity.update_value('x', self.state_time, self.velocity[0])
            self.plot_velocity.update_value('y', self.state_time, self.velocity[1])
            self.plot_velocity.update_value('z', self.state_time, self.velocity[2])

        if self.plot_acceleration_bias is not None:
            self.plot_acceleration_bias.update_value('x', self.state_time, self.acceleration_bias[0])
            self.plot_acceleration_bias.update_value('y', self.state_time, self.acceleration_bias[1])
            self.plot_acceleration_bias.update_value('z', self.state_time, self.acceleration_bias[2])

        if self.plot_scale is not None:
            self.plot_scale.update_value('scale', self.state_time, self.scale)

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
            self._widget.dyn_rec_frame.setEnabled(0)

    def _state_subscriber_callback(self, input):
        # save current values for plotting in timerupdate
        if self.plot_start_time is -1:
            self.plot_start_time = input.header.stamp.to_sec()

        self.state_time = input.header.stamp.to_sec() - self.plot_start_time
        self.position = [input.state[0], input.state[1], input.state[2]]
        self.velocity = [input.state[3], input.state[4], input.state[5]]
        self.acceleration_bias = [input.state[6], input.state[7], input.state[8]]
        self.scale = input.state[9]

    def _status_subscriber_callback(self, input):
        # save current values for widget updates in timerupdate
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

    def _reset_plots(self):
        # Set the "Start" Button to "Reset"
        self._widget.start_reset_plot_button.setText("Reset")
        self.plot_start_time = -1

        # Battery Voltage Plot
        # Check if plot already exists and remove it:
        if self.plot_battery_voltage is not None:
            self._widget.plot_battery_voltage_layout.removeWidget(self.plot_battery_voltage)
            self.plot_battery_voltage.close()
        # Crate the new plot:
        self.plot_battery_voltage = QwtDataPlot(self._widget)
        self._widget.plot_battery_voltage_layout.addWidget(self.plot_battery_voltage)
        self.plot_battery_voltage.add_curve('voltage', 'Voltage', [0], [0])

        # Position Plot
        # Check if plot already exists and remove it:
        if self.plot_position is not None:
            self._widget.plot_position_layout.removeWidget(self.plot_position)
            self.plot_position.close()
        # Crate the new plot:
        self.plot_position = QwtDataPlot(self._widget)
        self._widget.plot_position_layout.addWidget(self.plot_position)
        self.plot_position.add_curve('x', 'x-position', [0], [0])
        self.plot_position.add_curve('y', 'y-position', [0], [0])
        self.plot_position.add_curve('z', 'z-position', [0], [0])

        # Velocity Plot
        # Check if plot already exists and remove it:
        if self.plot_velocity is not None:
            self._widget.plot_velocity_layout.removeWidget(self.plot_velocity)
            self.plot_velocity.close()
        # Crate the new plot:
        self.plot_velocity = QwtDataPlot(self._widget)
        self._widget.plot_velocity_layout.addWidget(self.plot_velocity)
        self.plot_velocity.add_curve('x', 'x-velocity', [0], [0])
        self.plot_velocity.add_curve('y', 'y-velocity', [0], [0])
        self.plot_velocity.add_curve('z', 'z-velocity', [0], [0])

        # Acceleration Bias Plot
        # Check if plot already exists and remove it:
        if self.plot_acceleration_bias is not None:
            self._widget.plot_acceleration_bias_layout.removeWidget(self.plot_acceleration_bias)
            self.plot_acceleration_bias.close()
        # Crate the new plot:
        self.plot_acceleration_bias = QwtDataPlot(self._widget)
        self._widget.plot_acceleration_bias_layout.addWidget(self.plot_acceleration_bias)
        self.plot_acceleration_bias.add_curve('x', 'x-acc-bias', [0], [0])
        self.plot_acceleration_bias.add_curve('y', 'y-acc-bias', [0], [0])
        self.plot_acceleration_bias.add_curve('z', 'z-acc-bias', [0], [0])

        # Scale Plot
        # Check if plot already exists and remove it:
        if self.plot_scale is not None:
            self._widget.plot_scale_layout.removeWidget(self.plot_scale)
            self.plot_scale.close()
        # Crate the new plot:
        self.plot_scale = QwtDataPlot(self._widget)
        self._widget.plot_scale_layout.addWidget(self.plot_scale)
        self.plot_scale.add_curve('scale', 'visual scale', [0], [0])

    def _scale_slider_change(self, value):
        self._widget.scale_spin_box.setValue(value/100)

    def _scale_box_change(self, value):
        self._widget.scale_slider.setValue(value*100)

    def _height_slider_change(self, value):
        self._widget.height_spin_box.setValue(value/100)

    def _height_box_change(self, value):
        self._widget.height_slider.setValue(value*100)
    
    def shutdown_plugin(self):
        self._slow_timer.stop()
        self._fast_timer.stop()
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