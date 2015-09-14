#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Modified July 2013 by Georg Wiedebach

# -*- coding: utf-8 -*-
from __future__ import division
import math
import sys


from python_qt_binding.QtCore import QEvent, QPointF, Qt, SIGNAL, Signal, Slot
from python_qt_binding.QtGui import QPen, QVector2D
import Qwt

from numpy import arange, zeros, concatenate

class LineStyle:
    Solid = Qt.SolidLine
    Dashed = Qt.DashLine

# create real QwtDataPlot class
class QwtDataPlot(Qwt.QwtPlot):
    mouseCoordinatesChanged = Signal(QPointF)
    _colors = { LineStyle.Solid: [Qt.red, Qt.green, Qt.blue, Qt.magenta],
                LineStyle.Dashed: [Qt.red, Qt.green, Qt.blue, Qt.magenta] } #[Qt.darkRed, Qt.darkGreen, Qt.darkBlue, Qt.darkMagenta]
    _num_value_saved = 5000
    _num_values_ploted = 1000

    def __init__(self, *args):
        if len(args) == 2:
            super(QwtDataPlot, self).__init__(Qwt.QwtText(args[0]), args[1])
        else:
            super(QwtDataPlot, self).__init__(*args)
        self.setCanvasBackground(Qt.white)
        self.insertLegend(Qwt.QwtLegend(), Qwt.QwtPlot.BottomLegend)

        self._curves = {}
        self._data_offset_x = 0
        self._canvas_offset_x = 0
        self._canvas_offset_y = 0
        self._last_canvas_x = 0
        self._last_canvas_y = 0
        self._pressed_canvas_y = 0
        self._last_click_coordinates = None
        self._color_indexes = { LineStyle.Solid: 0, LineStyle.Dashed:0 }

        marker_axis_y = Qwt.QwtPlotMarker()
        marker_axis_y.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        marker_axis_y.setLineStyle(Qwt.QwtPlotMarker.HLine)
        marker_axis_y.setYValue(0.0)
        marker_axis_y.attach(self)

        self._picker = Qwt.QwtPlotPicker(
            Qwt.QwtPlot.xBottom, Qwt.QwtPlot.yLeft, Qwt.QwtPicker.PolygonSelection,
            Qwt.QwtPlotPicker.PolygonRubberBand, Qwt.QwtPicker.AlwaysOn, self.canvas()
        )
        self._picker.setRubberBandPen(QPen(self._colors[LineStyle.Solid][-1]))
        self._picker.setTrackerPen(QPen(self._colors[LineStyle.Solid][-1]))

        # Initialize data
        self._time_axis = arange(0, 10)
        self._canvas_display_height = 1000
        self._canvas_display_width = self.canvas().width()
        self._data_offset_x = self._num_value_saved - self._num_values_ploted
        self.redraw()
        self.move_canvas(0, 0)
        self.canvas().setMouseTracking(True)
        self.canvas().installEventFilter(self)

    def eventFilter(self, _, event):
        if event.type() == QEvent.MouseButtonRelease:
            x = self.invTransform(Qwt.QwtPlot.xBottom, event.pos().x())
            y = self.invTransform(Qwt.QwtPlot.yLeft, event.pos().y())
            self._last_click_coordinates = QPointF(x, y)
        elif event.type() == QEvent.MouseMove:
            x = self.invTransform(Qwt.QwtPlot.xBottom, event.pos().x())
            y = self.invTransform(Qwt.QwtPlot.yLeft, event.pos().y())
            coords = QPointF(x, y)
            if self._picker.isActive() and self._last_click_coordinates is not None:
                toolTip = 'origin x: %.5f, y: %.5f' % (self._last_click_coordinates.x(), self._last_click_coordinates.y())
                delta = coords - self._last_click_coordinates
                toolTip += '\ndelta x: %.5f, y: %.5f\nlength: %.5f' % (delta.x(), delta.y(), QVector2D(delta).length())
            else:
                toolTip = 'buttons\nleft: measure\nmiddle: move\nright: zoom x/y\nwheel: zoom y'
            self.setToolTip(toolTip)
            self.mouseCoordinatesChanged.emit(coords)
        return False

    def log(self, level, message):
        self.emit(SIGNAL('logMessage'), level, message)

    def resizeEvent(self, event):
        #super(QwtDataPlot, self).resizeEvent(event)
        Qwt.QwtPlot.resizeEvent(self, event)
        self.rescale()

    def add_curve(self, curve_id, curve_name, values_x, values_y, line_thickness=1.0, line_style=LineStyle.Solid):
        curve_id = str(curve_id)
        if self._curves.get(curve_id):
            return
        curve_object = Qwt.QwtPlotCurve(curve_name)
        curve_object.attach(self)
        line_color = self._colors[line_style][self._color_indexes[line_style] % len(self._colors[line_style])]
        curve_object.setPen(QPen(line_color, line_thickness, line_style))
        self._color_indexes[line_style] += 1
        self._curves[curve_id] = {
            'name': curve_name,
            'data': zeros(self._num_value_saved),
            'time': zeros(self._num_value_saved),
            'object': curve_object,
        }

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._curves[curve_id]['object'].hide()
            self._curves[curve_id]['object'].attach(None)
            del self._curves[curve_id]['object']
            del self._curves[curve_id]

    @Slot(str, list, list)
    def update_values(self, curve_id, values_x, values_y):
        for value_x, value_y in zip(values_x, values_y):
            self.update_value(curve_id, value_x, value_y)

    @Slot(str, float, float)
    def update_value(self, curve_id, value_x, value_y):
        curve_id = str(curve_id)
        # update data plot
        if curve_id in self._curves:
            # TODO: use value_x as timestamp
            self._curves[curve_id]['data'] = concatenate((self._curves[curve_id]['data'][1:], self._curves[curve_id]['data'][:1]), 1)
            self._curves[curve_id]['data'][-1] = float(value_y)
            self._curves[curve_id]['time'] = concatenate((self._curves[curve_id]['time'][1:], self._curves[curve_id]['time'][:1]), 1)
            self._curves[curve_id]['time'][-1] = float(value_x)

    def redraw(self):
        for curve_id in self._curves.keys():
            x_data = self._time_axis
            x_data = self._curves[curve_id]['time'][self._data_offset_x: -1]
            y_data = self._curves[curve_id]['data'][self._data_offset_x: -1]

            self._curves[curve_id]['object'].setData(x_data, y_data)
            #self._curves[curve_id]['object'].setStyle(Qwt.QwtPlotCurve.CurveStyle(3))

        self.replot()

    def rescale(self):
        y_num_ticks = self.height() / 40
        y_lower_limit = self._canvas_offset_y - (self._canvas_display_height / 2)
        y_upper_limit = self._canvas_offset_y + (self._canvas_display_height / 2)

        # calculate a fitting step size for nice, round tick labels, depending on the displayed value area
        y_delta = y_upper_limit - y_lower_limit
        exponent = int(math.log10(y_delta))
        presicion = -(exponent - 2)
        y_step_size = round(y_delta / y_num_ticks, presicion)

        self.setAxisScale(Qwt.QwtPlot.yLeft, y_lower_limit, y_upper_limit, y_step_size)

        self.setAxisScale(Qwt.QwtPlot.xBottom, self._time_axis[0], self._time_axis[-1]+1)
        self.redraw()

    def rescale_axis_x(self, delta__x):
        new_len = len(self._time_axis) + delta__x
        new_len = max(10, min(new_len, self._num_value_saved))
        self._time_axis = arange(new_len)
        self._data_offset_x = max(0, min(self._data_offset_x, self._num_value_saved - len(self._time_axis)))
        self.rescale()

    def scale_axis_y(self, min_value, max_value):
        self._canvas_display_height = max_value - min_value
        self._canvas_offset_y = (max_value + min_value)/2
        self.rescale()

    def rescale_axis_y(self):
        y_min = [-0.1]
        y_max = [0.1]
        for curve_id in self._curves.keys():
            y_min.append(min(self._curves[curve_id]['data']))
            y_max.append(max(self._curves[curve_id]['data']))
            time = self._curves[curve_id]['time'][-1]

        min_value = min(y_min)
        max_value = max(y_max)

        if time > 10:
            self._time_axis = arange(time-10, time)

        self._canvas_display_height = max_value - min_value
        self._canvas_offset_y = (max_value + min_value)/2
        self.rescale()

    def move_canvas(self, delta_x, delta_y):
        self._data_offset_x += delta_x * len(self._time_axis) / float(self.canvas().width())
        self._data_offset_x = max(0, min(self._data_offset_x, self._num_value_saved - len(self._time_axis)))
        self._canvas_offset_x += delta_x * self._canvas_display_width / self.canvas().width()
        self._canvas_offset_y += delta_y * self._canvas_display_height / self.canvas().height()
        self.rescale()

    def mousePressEvent(self, event):
        self._last_canvas_x = event.x() - self.canvas().x()
        self._last_canvas_y = event.y() - self.canvas().y()
        self._pressed_canvas_y = event.y() - self.canvas().y()

    def mouseMoveEvent(self, event):
        canvas_x = event.x() - self.canvas().x()
        canvas_y = event.y() - self.canvas().y()
        if event.buttons() & Qt.MiddleButton:  # middle button moves the canvas
            delta_x = self._last_canvas_x - canvas_x
            delta_y = canvas_y - self._last_canvas_y
            self.move_canvas(delta_x, delta_y)
        elif event.buttons() & Qt.RightButton:   # right button zooms
            zoom_factor = max(-0.6, min(0.6, (self._last_canvas_y - canvas_y) / 20.0 / 2.0))
            delta_y = (self.canvas().height() / 2.0) - self._pressed_canvas_y
            self.move_canvas(0, zoom_factor * delta_y * 1.0225)
            self.scale_axis_y(0, max(0.005, self._canvas_display_height - (zoom_factor * self._canvas_display_height)))
            self.rescale_axis_x(self._last_canvas_x - canvas_x)
        self._last_canvas_x = canvas_x
        self._last_canvas_y = canvas_y

    def wheelEvent(self, event):  # mouse wheel zooms the y-axis
        canvas_y = event.y() - self.canvas().y()
        zoom_factor = max(-0.6, min(0.6, (event.delta() / 120) / 6.0))
        delta_y = (self.canvas().height() / 2.0) - canvas_y
        self.move_canvas(0, zoom_factor * delta_y * 1.0225)
        self.scale_axis_y(0, max(0.0005, self._canvas_display_height - zoom_factor * self._canvas_display_height))


if __name__ == '__main__':
    from python_qt_binding.QtGui import QApplication

    app = QApplication(sys.argv)
    plot = QwtDataPlot()
    plot.resize(700, 500)
    plot.show()
    plot.add_curve(0, '(x/500)^2')
    plot.add_curve(1, 'sin(x / 20) * 500')
    for i in range(plot._num_value_saved):
        plot.update_value(0, (i / 500.0) * (i / 5.0))
        plot.update_value(1, math.sin(i / 20.0) * 500)

    sys.exit(app.exec_())
