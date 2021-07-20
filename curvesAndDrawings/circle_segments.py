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
from BackgroundControl.background_control import choosingBackground


class circSegment:
    
    def __init__(self,center_xcoord,center_ycoord,radius,startAngle,endAngle,puntos=500,backgroundColor="Black"):
        #startAngle -> clockwise -> endAngle
        #startAngle AND endAngle are input in radians
        backgroundColor = choosingBackground(backgroundColor)
        self.color = backgroundColor.white
        self.white = backgroundColor.white
        self.black = backgroundColor.black
        self.center_xcoord = center_xcoord
        self.center_ycoord = center_ycoord
        self.radius = radius
        self.startAngle = -16*180*startAngle/numpy.pi # THE UNIT IN QtGui.QGraphicsEllipseItem IS 1/16 OF A DEGREE
        self.endAngle = -16*180*endAngle/numpy.pi # THE UNIT IN QtGui.QGraphicsEllipseItem IS 1/16 OF A DEGREE
        self.spanAngle = self.endAngle - self.startAngle
        self.lowerLeftCorner_xcoord = self.center_xcoord - self.radius
        self.lowerLeftCorner_ycoord = self.center_ycoord - self.radius
        
        # self.startAngleRad = startAngle
        # self.endAngleRad = endAngle
        # self.puntosADibujar = puntos
        # self.x_coord = self.center_xcoord + self.radius*numpy.cos(numpy.linspace(self.startAngleRad,self.endAngleRad,self.puntosADibujar))
        # self.y_coord=  self.center_ycoord + self.radius*numpy.sin(numpy.linspace(self.startAngleRad,self.endAngleRad,self.puntosADibujar))
        # self.goodSegment = pg.PlotCurveItem(self.x_coord,self.y_coord, pen=pg.mkPen('k',width=2.5), clickable=True)
        # self.complementarySegment = self.goodSegment

        self.goodSegment = QtGui.QGraphicsEllipseItem(QtCore.QRectF(self.lowerLeftCorner_xcoord, self.lowerLeftCorner_ycoord, 2*self.radius, 2*self.radius))        
        self.goodSegment.setStartAngle(self.startAngle)
        self.goodSegment.setSpanAngle(self.spanAngle)
        self.goodSegment.setPen(pg.mkPen(color=self.white, width=2))
        self.complementarySegment = QtGui.QGraphicsEllipseItem(QtCore.QRectF(self.lowerLeftCorner_xcoord, self.lowerLeftCorner_ycoord, 2*self.radius, 2*self.radius))
        self.complementarySegment.setStartAngle(self.endAngle)
        self.complementarySegment.setPen(pg.mkPen(color=self.black, width=2))
        self.complementarySegment.setSpanAngle(16*numpy.sign(self.spanAngle)*360-self.spanAngle)

        self.whitePieces = [self.goodSegment]
        self.blackPieces= [self.complementarySegment]
        self.bluePieces =[]
        self.redPieces = []
        self.cyanPieces = []
        self.yellowPieces = []
        self.magentaPieces = []
        self.greenPieces = []




 


        
        
        

        
        
