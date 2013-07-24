from __future__ import division

import os
import rospy
import rospkg

# Import all necessary message types:
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import TransformStamped

# Qt related imports:
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget, QColor


class CopterPlugin(Plugin):

    scale = 0
    voltage = 100

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

        # Create Publisher and Subscriber
        self._publisher = rospy.Publisher('value', Int16)
        self._subscriber = rospy.Subscriber('voltage', Int16, self._sub_callback)

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
        msg = Int16()
        msg.data = self.scale*100
        self._publisher.publish(msg)

    def _sub_callback(self, input):
        self.voltage = input.data

    def _timer_update(self):
        self._widget.battery_voltage.setValue(self.voltage)
        if self.voltage < 40:
            self._widget.battery_voltage_alert.setVisible(1)
        else:
            self._widget.battery_voltage_alert.setVisible(0)
    
    def shutdown_plugin(self):
        if self._publisher is not None:
            self._publisher.unregister()
        self._publisher = None
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