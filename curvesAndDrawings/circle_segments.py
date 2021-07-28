#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 13 14:10:44 2021

@author: caesar
"""


from PyQt5 import QtGui#, QtCore
import numpy
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# Maybe it would be useful to transofrm cicle_segments into a curvesAndDrawings module with some other basic shapes

class circSegment:

    def __init__(self,center_xcoord,center_ycoord,radius,startAngle,endAngle):
        #startAngle -> clockwise -> endAngle
        #startAngle AND endAngle are input in radians
        self.center_xcoord = center_xcoord
        self.center_ycoord = center_ycoord
        self.radius = radius
        self.startAngle = 16*180*startAngle/numpy.pi # THE UNIT IN QtGui.QGraphicsEllipseItem IS 1/16 OF A DEGREE
        self.endAngle = 16*180*endAngle/numpy.pi # THE UNIT IN QtGui.QGraphicsEllipseItem IS 1/16 OF A DEGREE
        self.spanAngle = self.endAngle - self.startAngle
        self.lowerLeftCorner_xcoord = self.center_xcoord - self.radius
        self.lowerLeftCorner_ycoord = self.center_ycoord - self.radius

        self.goodSegment = QtGui.QGraphicsEllipseItem(QtCore.QRectF(self.lowerLeftCorner_xcoord, self.lowerLeftCorner_xcoord, 2*self.radius, 2*self.radius))
        self.goodSegment.setPen(pg.mkPen(color = 'k',width=2.5))
        # self.PlotWidgetIn_pageUHP.addItem(self.goodSegment)
        self.goodSegment.setStartAngle(self.startAngle)
        self.goodSegment.setSpanAngle(self.spanAngle)
        self.complementarySegment = QtGui.QGraphicsEllipseItem(QtCore.QRectF(self.lowerLeftCorner_xcoord, self.lowerLeftCorner_xcoord, 2*self.radius, 2*self.radius))
        self.complementarySegment.setPen(pg.mkPen(color = 'w',width=2.5))
        # self.PlotWidgetIn_pageUHP.addItem(self.complementarySegment)
        self.complementarySegment.setStartAngle(self.endAngle)
        self.complementarySegment.setSpanAngle(16*numpy.sign(self.spanAngle)*360-self.spanAngle)


class line():

    def __init__(self, startPoint, endPoint): 
        self.line = QtGui.QGraphicsLineItem(startPoint.real, startPoint.imag, endPoint.real, endPoint.imag) # Do we need to add "QGraphicsItem *parent = nullptr" as the last parameter ?
