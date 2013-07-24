from __future__ import division

import os
import rospy
import rospkg
import dynamic_reconfigure.client
from rosplot import ROSData, RosPlotException

# Import all necessary message types:
from std_msgs.msg import Float32, Int16, String
from geometry_msgs.msg import TransformStamped

# Qt related imports:
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget, QColor, QLabel
from mat_data_plot import MatDataPlot


class CopterPlugin(Plugin):

    scale = 0
    voltage = 50
    flight_mode_ll = "none"
    plot_buffer_x = 0
    plot_buffer_y = 0

    def __init__(self, context):
        super(CopterPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CopterPlugin')

        print "Copter GUI started"

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_copter'), 'resource', 'CopterPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CopterPluginUi')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Add the Plot to the GUI
        width = self._widget.plots.width()
        height = self._widget.plots.height()
        self._widget.plots = MatDataPlot(self._widget.plots)
        self._widget.plots.setGeometry(0, 0, width, height)
        self._widget.plots.show()

        # Create Publisher and Subscriber
        # self._publisher = rospy.Publisher('value', Int16)
        self._subscriber = rospy.Subscriber('voltage', Int16, self._sub_callback)
        self._subscriber = rospy.Subscriber('flight_mode', String, self._string_callback)
        self._subscriber = rospy.Subscriber('vicon/auk/auk', TransformStamped, self._transform_callback)
        self._start_time = rospy.get_time()
        print self._start_time
        self._widget.plots.add_curve('plot1', 'Testplot', [self.plot_buffer_x], [self.plot_buffer_y])

        # Bring up dynamic reconfigure for EKF init
        #self._client = dynamic_reconfigure.client.Client("pose_sensor", timeout=2)

        # Initialize Timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._timer_update)
        self._timer.start(50)

        # Initialize Widgets
        self._widget.gps_status.setText("Hallo")
        self._widget.battery_voltage_alert.setVisible(0)
        self._widget.scaleSlider.valueChanged.connect(self._on_slider_changed)
        self._widget.scaleBox.valueChanged.connect(self._on_box_changed)
        self._widget.scaleApply.clicked.connect(self._on_button_click)

    def _on_slider_changed(self, var):
        var = var/100
        self._widget.scaleBox.setValue(var)
        self.scale = var

    def _on_box_changed(self, var):
        self._widget.scaleSlider.setValue(var*100)
        self.scale = var

    def _on_button_click(self):
        #self._client.update_configuration({"scale_init":self.scale*100})
        pass

    def _sub_callback(self, input):
        self.voltage = input.data

    def _string_callback(self, input):
        self.flight_mode_ll = input.data

    def _transform_callback(self, input):
        self.plot_buffer_y = input.transform.translation.x
        self.plot_buffer_x = input.header.stamp.to_sec()

    def _timer_update(self):
        self._widget.battery_voltage.setValue(self.voltage)
        if self.voltage < 40:
            self._widget.battery_voltage_alert.setVisible(1)
        else:
            self._widget.battery_voltage_alert.setVisible(0)

        self._widget.flight_mode_ll.setText(self.flight_mode_ll)
        
        self._widget.plots.update_values('plot1', [self.plot_buffer_x], [self.plot_buffer_y])
        self._widget.plots.redraw()
    
    def shutdown_plugin(self):
        self._timer.stop()

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