from __future__ import division

import os
import rospy
import rospkg
import rosservice

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

# TODO: Implement InputWidgets for all possible Input Types
# So far only double/float32 is done. Which types are needed?

# Widget that allows the user to input a double:
class DoubleInputWidget():
    def __init__(self, context, parameter_name):
    	self._parameter_name = parameter_name

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_copter'), 'resource', 'DoubleInputWidget.ui')
        loadUi(ui_file, self._widget)
        context.addWidget(self._widget)

        self._widget.input_label.setText("Change " + self._parameter_name)
        self._widget.input_slider.valueChanged.connect(self._slider_change)
        self._widget.input_spin_box.valueChanged.connect(self._box_change)

    def _slider_change(self, value):
        self._widget.input_spin_box.setValue(value/100)

    def _box_change(self, value):
        self._widget.input_slider.setValue(value*100)

    def getValue(self):
    	return self._widget.input_spin_box.value()

    def getName(self):
    	return self._parameter_name

# Widget for a msf-initialize service. It creates the input masks for
# all request slots (e.g. two doubles) and handles the service call.
class SrvWidget():
    def __init__(self, context, service_name):
        self._service_name = service_name
        self._service = rosservice.get_service_class_by_name(self._service_name)
        self._service_proxy = rospy.ServiceProxy(self._service_name, self._service)

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_copter'), 'resource', 'SrvWidget.ui')
        loadUi(ui_file, self._widget)
        context.addWidget(self._widget)

        self._widget.service_label.setText("Service " + self._service_name)

        self._request = self._service._request_class()
        self._inputs = []

        for slot_name, type_name in zip(self._request.__slots__, self._request._slot_types):
            if type_name is "float32":
                self._inputs.append(DoubleInputWidget(self._widget.input_container, slot_name))
            else:
                print "Service input type", type_name, "needs to be implemented!"
                print "Service", self._service_name, "is not available."
                self._widget.close()

        self._widget.apply_button.clicked.connect(self._init_msf)

    def _init_msf(self):
        counter = 0
        for slot_name in self._request.__slots__:
            while slot_name is not self._inputs[counter].getName():
            	counter = counter + 1

            value = self._inputs[counter].getValue()
            setattr(self._request, slot_name, value)
            counter = 0

        try:
            self._response = self._service_proxy(self._request)
            if hasattr(self._response, 'result'):
                print self._response.result
        except rospy.ServiceException:
            print "Service call failed"

