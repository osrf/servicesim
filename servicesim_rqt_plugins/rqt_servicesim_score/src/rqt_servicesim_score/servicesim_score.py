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

from servicesim_competition.msg import Score
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget


class ServicesimScore(Plugin):

    def __init__(self, context):
        super(ServicesimScore, self).__init__(context)
        self.setObjectName('ServicesimScore')
        rp = rospkg.RosPack()

        self._widget = QWidget()
        ui_file = os.path.join(
            rp.get_path('rqt_servicesim_score'), 'resource', 'ServicesimScore.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ServicesimScoreUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        rospy.Subscriber("/servicesim/score", Score, self.callback)

    def callback(self,data):
        self.score = data.score
        self._widget.score_value.setText(
        '%0.4f' %(self.score))
