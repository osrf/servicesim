#! /usr/bin/env python

# Copyright (C) 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import division
import os
import rospkg
import rospy

from std_msgs.msg import Float64
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget

class PanTilt(Plugin):

    slider_factor = 1000.0

    def __init__(self, context):
        super(PanTilt, self).__init__(context)
        self.setObjectName('PanTilt')

        rp = rospkg.RosPack()
        self.PAN_TOPIC = "/servicebot/head_pan_position_controller/command"
        self._pan_publisher = rospy.Publisher(self.PAN_TOPIC, Float64, queue_size=10)
        self.TILT_TOPIC = "/servicebot/head_tilt_position_controller/command"
        self._tilt_publisher = rospy.Publisher(self.TILT_TOPIC, Float64, queue_size=10)

        self._widget = QWidget()
        ui_file = os.path.join(rp.get_path('rqt_servicebot_pan_tilt'), 'resource', 'PanTilt.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('PanTiltUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        #For pan angular slider
        self._widget.pan_angular_slider.valueChanged.connect(
            self._on_pan_angular_slider_changed)

        self._widget.increase_pan_angular_push_button.pressed.connect(
            self._on_strong_increase_pan_angular_pressed)
        self._widget.decrease_pan_angular_push_button.pressed.connect(
            self._on_strong_decrease_pan_angular_pressed)
        self._widget.reset_pan_angular_push_button.pressed.connect(
            self._on_reset_pan_angular_push_button)

        #For tilt angular slider
        self._widget.tilt_angular_slider.valueChanged.connect(
            self._on_tilt_angular_slider_changed)

        self._widget.increase_tilt_angular_push_button.pressed.connect(
            self._on_strong_increase_tilt_angular_pressed)
        self._widget.decrease_tilt_angular_push_button.pressed.connect(
            self._on_strong_decrease_tilt_angular_pressed)
        self._widget.reset_tilt_angular_push_button.pressed.connect(
            self._on_reset_tilt_angular_push_button)

        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

    @Slot(str)
    def _on_parameter_changed(self):
        self._send_command(
            self._widget.pan_angular_slider.value()/PanTilt.slider_factor, self._widget.tilt_angular_slider.value()/PanTilt.slider_factor)

    def _send_command(self, pan_angular, tilt_angular):
        if self._pan_publisher is None and self._tilt_publisher is None:
            return
        pan_msg = Float64()
        pan_msg.data = pan_angular
        tilt_msg = Float64()
        tilt_msg.data = tilt_angular
        # Only send the zero command once so other devices can take control
        if pan_angular == 0 and tilt_angular ==0:
            if not self.zero_cmd_sent:
                self.zero_cmd_sent = True
                self._pan_publisher.publish(pan_msg)
                self._tilt_publisher.publish(tilt_msg)
        else:
            self.zero_cmd_sent = False
            self._pan_publisher.publish(pan_msg)
            self._tilt_publisher.publish(tilt_msg)

    #Functions for pan slider
    def _on_pan_angular_slider_changed(self):
        self._widget.current_pan_angular_label.setText(
            '%0.2f rad' % (self._widget.pan_angular_slider.value() / PanTilt.slider_factor))
        self._on_parameter_changed()

    def _on_strong_decrease_pan_angular_pressed(self):
        self._widget.pan_angular_slider.setValue(
            self._widget.pan_angular_slider.value() - self._widget.pan_angular_slider.pageStep())

    def _on_strong_increase_pan_angular_pressed(self):
        self._widget.pan_angular_slider.setValue(
            self._widget.pan_angular_slider.value() + self._widget.pan_angular_slider.pageStep())

    def _on_reset_pan_angular_push_button(self):
        self._widget.pan_angular_slider.setValue(0)

    def _on_max_pan_angular_changed(self, value):
        self._widget.pan_angular_slider.setMaximum(
            value * PanTilt.slider_factor)

    def _on_min_pan_angular_changed(self, value):
        self._widget.pan_angular_slider.setMinimum(
            value * PanTilt.slider_factor)

    #Functions for tilt slider
    def _on_tilt_angular_slider_changed(self):
        self._widget.current_tilt_angular_label.setText(
            '%0.2f rad' % (self._widget.tilt_angular_slider.value() / PanTilt.slider_factor))
        self._on_parameter_changed()

    def _on_strong_decrease_tilt_angular_pressed(self):
        self._widget.tilt_angular_slider.setValue(
            self._widget.tilt_angular_slider.value() - self._widget.tilt_angular_slider.pageStep())

    def _on_strong_increase_tilt_angular_pressed(self):
        self._widget.tilt_angular_slider.setValue(
            self._widget.tilt_angular_slider.value() + self._widget.tilt_angular_slider.pageStep())

    def _on_reset_tilt_angular_push_button(self):
        self._widget.tilt_angular_slider.setValue(0)

    def _on_max_tilt_angular_changed(self, value):
        self._widget.tilt_angular_slider.setMaximum(
            value * PanTilt.slider_factor)

    def _on_min_tilt_angular_changed(self, value):
        self._widget.tilt_angular_slider.setMinimum(
            value * PanTilt.slider_factor)

    def _unregister_publisher(self):
        if self._pan_publisher is not None:
            self._pan_publisher.unregister()
            self._pan_publisher = None
        if self._tilt_publisher is not None:
            self._tilt_publisher.unregister()
            self._tilt_publisher = None

    def shutdown_plugin(self):
        self._unregister_publisher()
