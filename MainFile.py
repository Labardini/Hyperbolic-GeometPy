#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  6 15:53:06 2018
@author: daniellabardini
"""



from PyQt5 import QtGui, QtWidgets
#from PyQt5.QtCore import *
#from PyQt5.QtGui import *
import sys

import numpy


import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

from pyqtgraph.ptime import time

#### FOR HYPERBOLOID
#import pyqtgraph.opengl as gl
####





import Window
import draggableDots as dD

from exception_handling import Maybe
from exception_handling import myInputError

from Maths.CP_Maths import extended_complex_plane_CP
from Maths.CP_Maths import Steiner_grids_CP
from Maths.CP_Maths import Mobius_CP
from Maths.HP_Maths import UHP_HP
from Maths.HP_Maths import PD_HP

#### SOME FUNCTIONS IMPORTED FROM Maths.CP_Maths.extended_complex_plane_CP
oo = extended_complex_plane_CP.numpyExtendedComplexPlane().oo
coords = extended_complex_plane_CP.numpyExtendedComplexPlane().coords
isooInArgs = extended_complex_plane_CP.numpyExtendedComplexPlane().isooInArgs
isooInList = extended_complex_plane_CP.numpyExtendedComplexPlane().isooInList
extendedValue = extended_complex_plane_CP.numpyExtendedComplexPlane().extendedValue
areAllDistinctArgs = extended_complex_plane_CP.numpyExtendedComplexPlane().areAllDistinctArgs
areAllDistinctList = extended_complex_plane_CP.numpyExtendedComplexPlane().areAllDistinctList
removeooFromArgs = extended_complex_plane_CP.numpyExtendedComplexPlane().removeooFromArgs
removeooFromList = extended_complex_plane_CP.numpyExtendedComplexPlane().removeooFromList
e_circumcenter_and_radius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius
areCollinear = extended_complex_plane_CP.numpyExtendedComplexPlane().areCollinear
####

j = 1j
oneClick = []
twoClicks = []
threeClicks = []
arbManyClicks = []
auxStorage = []
auxStorage2 = []
Clicks = [oneClick,twoClicks,threeClicks,arbManyClicks,auxStorage,auxStorage2]




        

       









class appMainWindow(QtWidgets.QDialog, Window.Ui_MainWindow):
    
    def __init__(self, parent=None):
        super(appMainWindow,self).__init__(parent)
        self.setupUi(self)
        
        self.timer = None # in some animations it will become QtCore.QTimer(self)
        
        ### pens
        self.blackPenWidth2 = pg.mkPen('k', width=2)
        self.redPenWidth2 = pg.mkPen('r', width=2)
        self.bluePenWidth2 = pg.mkPen('b', width=2)
        
        

##################
##################
##### AXES LIMITS for PlotWidgetIn_pageCP

        self.CP_absolute_lim = 100
        self.CP_xlim_left = -self.CP_absolute_lim
        self.CP_xlim_right = self.CP_absolute_lim
        self.CP_ylim_down = -self.CP_absolute_lim
        self.CP_ylim_up = self.CP_absolute_lim
        
        self.PlotWidgetIn_pageCP.setXRange(self.CP_xlim_left,self.CP_xlim_right)
        #self.PlotWidgetIn_pageCP.setLimits(yMin=0.0)
        self.PlotWidgetIn_pageCP.disableAutoRange()
#        self.graphicsView_2.setYRange(self.CP_ylim_down,self.CP_ylim_up)
        self.PlotWidgetIn_pageCP.setAspectLocked(1.0)
        self.CPdraggableDotsMobTransFromParameters = dD.draggableDot() # FIXED POINTS AS DRAGGABLE DOTS
        self.PlotWidgetIn_pageCP.addItem(self.CPdraggableDotsMobTransFromParameters)
        
        self.PlotWidgetIn_pageCPMobiusTransformations.setYRange(-2,2)
        #self.PlotWidgetIn_pageCPMobiusTransformations.setLimits(yMin=-2)
        #self.PlotWidgetIn_pageCPMobiusTransformations.disableAutoRange()
#        self.graphicsView_2.setYRange(self.CP_ylim_down,self.CP_ylim_up)
        self.PlotWidgetIn_pageCPMobiusTransformations.setAspectLocked(1.0)
        self.PlotWidgetIn_pageCPMobiusTransformations.disableAutoRange()
        # self.horLineCrosshairCPMT = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('k', width=1.5))
        # self.vertLineCrosshairCPMT = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('k', width=1.5))
        # self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.horLineCrosshairCPMT)
        # self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.vertLineCrosshairCPMT)        
        self.widthunitCircleCPMobTransFromParameters = 3
        self.unitCircleCPMobTransFromParameters = pg.PlotCurveItem(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen=pg.mkPen('b',width=self.widthunitCircleCPMobTransFromParameters), clickable=True)
        self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.unitCircleCPMobTransFromParameters)
        self.widthPosRealsCPMobTransFromParameters = 3
        self.PosRealsCPMobTransFromParameters = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('k', width=self.widthPosRealsCPMobTransFromParameters))
        self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.PosRealsCPMobTransFromParameters )
        self.ImagAxisCPMobTransFromParameters = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('k', width=self.widthPosRealsCPMobTransFromParameters))
        self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.ImagAxisCPMobTransFromParameters )
        #self.myPlotCurveItem = pg.PlotCurveItem([0, 2],[0,0],pen=pg.mkPen('b',width=3),clickable=True)
        #self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.myPlotCurveItem)
        self.CPdraggableDotsMobTransFromParametersParEllAlphaParameter = dD.unitCircledraggableDot() # ALPHA PARAMETER AS DRAGGABLE DOT FOR PARABOLIC OR ELLIPTIC TRANSFORMATION
        self.CPdraggableDotsMobTransFromParametersParEllAlphaParameter.setData(pos=numpy.array([[0,1]],dtype=float),pxMode=True)
        self.CPdraggableDotsMobTransFromParametersParHypAlphaParameter = dD.PosRealsdraggableDot() # ALPHA PARAMETER AS DRAGGABLE DOT FOR PARABOLIC OR HYPERBOLIC TRANSFORMATION
        self.CPdraggableDotsMobTransFromParametersParHypAlphaParameter.setData(pos=numpy.array([[2,0]],dtype=float),pxMode=True)
        self.CPdraggableDotsMobTransFromParametersParLoxAlphaParameter = dD.draggableDot() # ALPHA PARAMETER AS DRAGGABLE DOT FOR PARABOLIC OR LOXODROMIC TRANSFORMATION
        self.CPdraggableDotsMobTransFromParametersParLoxAlphaParameter.setData(pos=numpy.array([[1,1]],dtype=float),pxMode=True)
        self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.CPdraggableDotsMobTransFromParametersParEllAlphaParameter)
        self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.CPdraggableDotsMobTransFromParametersParHypAlphaParameter)
        self.PlotWidgetIn_pageCPMobiusTransformations.addItem(self.CPdraggableDotsMobTransFromParametersParLoxAlphaParameter)





##################
##################
##### AXES LIMITS for PlotWidgetIn_pageUHP
        self.UHP_absolute_lim = 100
        self.UHP_xlim_left = -self.UHP_absolute_lim
        self.UHP_xlim_right = self.UHP_absolute_lim
        #self.UHP_ylim_down = -self.CP_absolute_lim
        self.UHP_ylim_up = self.UHP_absolute_lim
        
        self.PlotWidgetIn_pageUHP.setXRange(self.UHP_xlim_left,self.UHP_xlim_right)
        self.PlotWidgetIn_pageUHP.setLimits(yMin=0)
        self.PlotWidgetIn_pageUHP.disableAutoRange()
#        self.graphicsView_2.setYRange(self.CP_ylim_down,self.CP_ylim_up)
        self.PlotWidgetIn_pageUHP.setAspectLocked(1.0)
        self.widthBoundingLineUHP = 5
        self.boundingLineUHP = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('k', width=self.widthBoundingLineUHP))
        self.PlotWidgetIn_pageUHP.addItem(self.boundingLineUHP)
        self.UHPdraggableDotsStaticGeodSegs = dD.UHPdraggableDot()
        self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDotsStaticGeodSegs)
        self.UHPdraggableDotsConvexHull = dD.UHPdraggableDot()
        self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDotsConvexHull)


        
        

        
  
        self.PlotWidgetIn_pageUHPGeodesicMotion.setXRange(-2,2)
        self.PlotWidgetIn_pageUHPGeodesicMotion.setYRange(-2,2)
        self.PlotWidgetIn_pageUHPGeodesicMotion.hideAxis('left')
        self.PlotWidgetIn_pageUHPGeodesicMotion.showAxis('right')
        self.PlotWidgetIn_pageUHPGeodesicMotion.disableAutoRange()
        self.PlotWidgetIn_pageUHPGeodesicMotion.setAspectLocked(1.0)
        self.widthLinesUHPGM = 2
        self.boundingCircleUHPGM = pg.PlotCurveItem(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen=pg.mkPen('m',width=self.widthLinesUHPGM), clickable=True)
        self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(self.boundingCircleUHPGM)
        self.horLineCrosshairUHPGM = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('k', width=self.widthLinesUHPGM))
        self.vertLineCrosshairUHPGM = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('k', width=self.widthLinesUHPGM))
        self.horLineUHPGM = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('m',width=self.widthLinesUHPGM))
        self.vertLineUHPGM = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('m',width=self.widthLinesUHPGM))
        self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(self.horLineCrosshairUHPGM, ignoreBounds = True)
        self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(self.vertLineCrosshairUHPGM, ignoreBounds = True)
        self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(self.horLineUHPGM, ignoreBounds = True)
        self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(self.vertLineUHPGM, ignoreBounds = True)
        
#        self.PlotWidgetIn_pageUHPGeodesicMotion.hide()



##################
##################
##### AXES LIMITS for PlotWidgetIn_pagePD
        self.PD_absolute_lim = 1
        self.PD_xlim_left = -self.PD_absolute_lim
        self.PD_xlim_right = self.PD_absolute_lim
        self.PD_ylim_down = -self.PD_absolute_lim
        self.PD_ylim_up = self.PD_absolute_lim
        
        self.PlotWidgetIn_pagePD.setXRange(self.PD_xlim_left,self.PD_xlim_right)
        self.PlotWidgetIn_pagePD.setYRange(self.PD_ylim_down,self.PD_ylim_up)
        self.PlotWidgetIn_pagePD.setLimits(xMin=-2,xMax=2,yMin=-2,yMax=2)
        self.PlotWidgetIn_pagePD.disableAutoRange()
#        self.graphicsView_2.setYRange(self.CP_ylim_down,self.CP_ylim_up)
        self.PlotWidgetIn_pagePD.setAspectLocked(1.0)
        self.PlotWidgetIn_pagePD.hideAxis("left")
        self.PlotWidgetIn_pagePD.hideAxis("bottom")
        self.boundingCirclePD = pg.PlotCurveItem(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen=pg.mkPen('k',width=2), clickable=True)
        self.PlotWidgetIn_pagePD.addItem(self.boundingCirclePD)
        self.PDdraggableDotsStaticGeodSegs = dD.PDdraggableDot()
        self.PlotWidgetIn_pagePD.addItem(self.PDdraggableDotsStaticGeodSegs)
        self.PDdraggableDotsConvexHull = dD.PDdraggableDot()
        self.PlotWidgetIn_pagePD.addItem(self.PDdraggableDotsConvexHull)
        self.PDdraggableDotsMidPtForSidePairing = dD.PDdraggableDot()
        self.PlotWidgetIn_pagePD.addItem(self.PDdraggableDotsMidPtForSidePairing)
        
        
#        self.PlotWidgetIn_pagePD.plot(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)),pen='k')
#        self.circROIPD = pg.CircleROI([-1, -1], [2, 2], pen=(4,100), maxBounds=[2,2])
#        self.PlotWidgetIn_pagePD.addItem(self.circROIPD)
#        self.circROIPD.removeHandle(0)


        self.PlotWidgetIn_pagePDGeodesicMotion.setXRange(-2,2)
        self.PlotWidgetIn_pagePDGeodesicMotion.setYRange(-2,2)
        self.PlotWidgetIn_pagePDGeodesicMotion.hideAxis('left')
        self.PlotWidgetIn_pagePDGeodesicMotion.showAxis('right')
        self.PlotWidgetIn_pagePDGeodesicMotion.disableAutoRange()
        self.PlotWidgetIn_pagePDGeodesicMotion.setAspectLocked(1.0)
        self.widthLinesPDGM = 2
        self.boundingCirclePDGM = pg.PlotCurveItem(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen=pg.mkPen('m',width=self.widthLinesPDGM), clickable=True)
        self.PlotWidgetIn_pagePDGeodesicMotion.addItem(self.boundingCirclePDGM)
        self.horLineCrosshairPDGM = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('k', width=self.widthLinesPDGM))
        self.vertLineCrosshairPDGM = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('k', width=self.widthLinesPDGM))
        self.horLinePDGM = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('m',width=self.widthLinesPDGM))
        self.vertLinePDGM = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('m',width=self.widthLinesPDGM))
        self.PlotWidgetIn_pagePDGeodesicMotion.addItem(self.horLineCrosshairPDGM, ignoreBounds = True)
        self.PlotWidgetIn_pagePDGeodesicMotion.addItem(self.vertLineCrosshairPDGM, ignoreBounds = True)
        self.PlotWidgetIn_pagePDGeodesicMotion.addItem(self.horLinePDGM, ignoreBounds = True)
        self.PlotWidgetIn_pagePDGeodesicMotion.addItem(self.vertLinePDGM, ignoreBounds = True)
















#####################
#####################
######## DISPLAY HYPERBOLOID
#
#
#        self.openGLWidget.opts['viewport'] =  (0, 0, 1100, 900)
#        #self.openGLWidget.showMaximized()
#        self.openGLWidget.setCameraPosition(distance=50)
#        
#        
#
#        ## Add a grid to the view
#        g = gl.GLGridItem()
#        #g.scale(2,2,1)
#        #g.setDepthValue(10)  # draw grid after surfaces since they may be translucent
#        g.setSize(50,50,50)
#        g.translate(20,0,-30)
#        self.openGLWidget.addItem(g)
#        
#                
#        x = numpy.linspace(-20, 20, 100)
#        y = numpy.linspace(-20, 20, 100)
#        z = numpy.sqrt((x.reshape(100,1) ** 2) + (y.reshape(1,100) ** 2) + 1)
##        x = numpy.linspace(-50, 50, 100)
##        y = numpy.linspace(-50, 50, 100)
##        z = numpy.sqrt((x.reshape(100,1) ** 2) + (y.reshape(1,100) ** 2) + 1)
#        p2 = gl.GLSurfacePlotItem(x=x, y=y, z=z, shader='shaded', color=(0.5, 0.5, 1, 1))
#        p2.translate(20,0,-30)
#        self.openGLWidget.addItem(p2)
#
























#####
##### Daniel's own configuration of some buttons, signals and slots,
##### for navigation through the app's pages
##### DEFINITIONS OF CONNECTIONS FOR SIGNALS
##### SEE BELOW FOR THE DEFINITIONS OF THE SIGNALS


        
        
        self.toolButtonHome.clicked.connect(self.effectOf_toolButtonHome)
        self.toolButtonCP.clicked.connect(self.effectOf_toolButtonCP)
        self.toolButtonHP.clicked.connect(self.effectOf_toolButtonHP)
#        self.toolButtonTS.clicked.connect(self.effectOf_toolButtonTS)
#        self.toolButtonMS.clicked.connect(self.effectOf_toolButtonMS)
#        self.toolButtonArt.clicked.connect(self.effectOf_toolButtonArt)
        
        
#        self.comboBox.currentIndexChanged.connect(self.chooseBackgroundColor)
        
        
        
#####
##### Daniel's own configuration of some buttons and signals that are supposed
##### to have a mathematical effect or an effect inside a specific page
##### DEFINITIONS OF CONNECTIONS FOR SIGNALS
##### SEE BELOW FOR THE DEFINITIONS OF THE SIGNALS

        self.pushButtonCPClearCanvas.clicked.connect(self.effectOf_pushButtonCPClearCanvas)

        self.pushButtonCPSGCommon.clicked.connect(self.effectOf_pushButtonCPSGCommon)
        self.pushButtonCPSGApollonius.clicked.connect(self.effectOf_pushButtonCPSGApollonius)
        self.pushButtonCPSGSteiner.clicked.connect(self.effectOf_pushButtonCPSGSteiner)
        self.horizontalSliderCPSGLoxodromesAngleTheta.sliderMoved.connect(self.effectOf_horizontalSliderCPSGLoxodromesAngleTheta)
        
        self.pushButtonCPMTFixedPoints.clicked.connect(self.effectOf_pushButtonCPMTFixedPoints)
        self.pushButtonCPMTStaticOrbitSinglePoint.clicked.connect(self.effectOf_pushButtonCPMTStaticOrbitSinglePoint)
#        self.pushButtonCPMTOrbitsRandomCircle.clicked.connect(self.effectOf_pushButtonCPMTOrbitsRandomCircle)
#        self.pushButtonCPMTOrbitsSteinerGrid.clicked.connect(self.effectOf_pushButtonCPMTOrbitsSteinerGrid)



        self.PlotWidgetIn_pageCP.scene().sigMouseClicked.connect(self.CPMTMobFromParamFixedPoints)
        #self.unitCircleCPMobTransFromParameters.sigClicked.connect(self.CPMTellHypAlphaParameter)

        #self.proxyCPMT = pg.SignalProxy(self.PlotWidgetIn_pageCPMobiusTransformations.scene().sigMouseMoved, rateLimit=60, slot=self.CPMTmouseMoved)
        self.CPdraggableDotsMobTransFromParametersParEllAlphaParameter.Dot.moved.connect(self.CPMTAlphaParameter)
        self.CPdraggableDotsMobTransFromParametersParHypAlphaParameter.Dot.moved.connect(self.CPMTAlphaParameter)
        self.CPdraggableDotsMobTransFromParametersParLoxAlphaParameter.Dot.moved.connect(self.CPMTAlphaParameter)
        






        self.pushButtonUHPClearCanvas.clicked.connect(self.effectOf_pushButtonUHPClearCanvas)
        self.radioButtonUHPBCGeodesicSegments.clicked.connect(self.deleteClicks)
        self.radioButtonUHPBCConvexHull.clicked.connect(self.deleteClicks)
        self.proxyUHP = pg.SignalProxy(self.PlotWidgetIn_pageUHP.scene().sigMouseMoved, rateLimit=60, slot=self.UHPmouseMoved)
        self.UHPdraggableDotsStaticGeodSegs.Dot.moved.connect(self.UHPBCDragGeodesicSegment)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPBCGeodesicSegmentStatic)
        self.UHPdraggableDotsConvexHull.Dot.moved.connect(self.UHPBCDragConvexHull)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPBCConvexHull)
        self.proxyUHPGM = pg.SignalProxy(self.PlotWidgetIn_pageUHPGeodesicMotion.scene().sigMouseMoved, rateLimit=60, slot=self.UHPGMmouseMoved)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPGMGeodesicSegmentAnimated)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPGMGeodesicRayConstantRapidityAnimated)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPCMCircMotionAntiClockwise)
#        self.pushButtonUHPCMCircMotionAntiClockwise.clicked.connect(self.effectOf_pushButtonUHPCMCircMotionAntiClockwise)
        self.pushButtonUHPIsomsComputePolygonAndFuchsianGroup.clicked.connect(self.UHPIsomsSpecificIdealPolygon)


        self.pushButtonPDClearCanvas.clicked.connect(self.effectOf_pushButtonPDClearCanvas)
        self.radioButtonPDBCGeodesicSegments.clicked.connect(self.deleteClicks)
        self.radioButtonPDBCConvexHull.clicked.connect(self.deleteClicks)        
#        self.proxyPD = pg.SignalProxy(self.circROIPD.sigHoverEvent, rateLimit=60, slot=self.PDmouseMoved)
        self.proxyPD = pg.SignalProxy(self.PlotWidgetIn_pagePD.scene().sigMouseMoved, rateLimit=100, slot=self.PDmouseMoved)
        self.PDdraggableDotsStaticGeodSegs.Dot.moved.connect(self.PDBCDragGeodesicSegment)
        self.PlotWidgetIn_pagePD.scene().sigMouseClicked.connect(self.PDBCGeodesicSegmentStatic)
        self.PDdraggableDotsConvexHull.Dot.moved.connect(self.PDBCDragConvexHull)
        self.PlotWidgetIn_pagePD.scene().sigMouseClicked.connect(self.PDBCConvexHull)
        self.proxyPDGM = pg.SignalProxy(self.PlotWidgetIn_pagePDGeodesicMotion.scene().sigMouseMoved, rateLimit=60, slot=self.PDGMmouseMoved)
        self.PlotWidgetIn_pagePD.scene().sigMouseClicked.connect(self.PDGMGeodesicSegmentAnimated)
        self.PlotWidgetIn_pagePD.scene().sigMouseClicked.connect(self.PDGMGeodesicRayConstantRapidityAnimated)
        self.pushButtonPDIsomsComputePolygonAndFuchsianGroup.clicked.connect(self.PDIsomsSpecificIdealPolygon)










#####
##### Daniel's own configuration of some buttons, signals and slots,
##### for navigation through the app's pages 
##### DEFINITIONS OF SIGNALS


        
    def effectOf_toolButtonHome(self):
        self.stackedWidgetAllPages.setCurrentIndex(0)
    
    def effectOf_toolButtonCP(self):
        self.stackedWidgetAllPages.setCurrentIndex(1)
    
    def effectOf_toolButtonHP(self):
        self.stackedWidgetAllPages.setCurrentIndex(2)
    
    def effectOf_toolButtonTS(self):
        self.stackedWidgetAllPages.setCurrentIndex(3)
    
    def effectOf_toolButtonMS(self):
        self.stackedWidgetAllPages.setCurrentIndex(4)
    
    def effectOf_toolButtonArt(self):
        self.stackedWidgetAllPages.setCurrentIndex(5)
        
    def deleteClicks(self):
        for clicked in Clicks:
            clicked.clear()
        
        
#    def chooseBackgroundColor(self):
#        text = self.comboBox.currentText()
#        if text == "White":
#            brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
#            #brush.setStyle(QtCore.Qt.NoBrush)
#            self.PlotWidgetIn_pageUHP.setBackgroundBrush(brush)
        
#######################
##### Function for plotting any point of the extended complex plane

    def plottingPointInExtendedPlane(self,point,pen,brush):
        P = extendedValue(point)
        if P != oo:
            pointForPlot = pg.ScatterPlotItem([P.real],[P.imag],pen=pen,brush=brush)
            self.PlotWidgetIn_pageCP.addItem(pointForPlot)
        else:
            self.radioButtonCPoo.setChecked(True)
        
        
#######################        
##### Function for plotting a line or circle through three points
##### without having to worry about collinearity or about one of the points being oo
##### Notice that it plots only in PlotWidgetIn_pageCP

    def PlottingCircleOrLineThrough3Points(self,p,q,r,color):
        P,Q,R = extendedValue(p),extendedValue(q),extendedValue(r)
        if areAllDistinctArgs(P,Q,R) == False:
            raise myInputError(str(P)+','+str(R)+','+str(Q),"The points must be distinct")
        else:
            if areCollinear(P,Q,R) == False:
                centerAndRadius = e_circumcenter_and_radius(P,Q,R)## comes in format [[x,y]],radius]
                center = centerAndRadius[0]
                radius = centerAndRadius[1]
                theta = numpy.linspace(0,2*numpy.pi,500)
                x_coord = center[0] + radius*numpy.cos(theta)
                y_coord = center[1] + radius*numpy.sin(theta)
                self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen=color)
            else:
                finitePoints = removeooFromArgs(P,Q,R)
                angleWithHorizontalLines = 180*numpy.angle(finitePoints[1]-finitePoints[0])/numpy.pi
                line = pg.InfiniteLine(pos = [finitePoints[0].real,finitePoints[0].imag], angle = angleWithHorizontalLines, pen=color)
                self.PlotWidgetIn_pageCP.addItem(line)
                
        
                
        
        
        
#####
##### Daniel's own configuration of some buttons and signals that are supposed
##### to have a mathematical effect
##### DEFINITIONS OF SIGNALS

    def effectOf_pushButtonCPClearCanvas(self):
        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
            self.timer = None
        self.PlotWidgetIn_pageCP.clear()
        self.radioButtonCPoo.setChecked(False)
        self.PlotWidgetIn_pageCP.setXRange(self.CP_xlim_left,self.CP_xlim_right)
        self.PlotWidgetIn_pageCP.setAspectLocked(1.0)
        self.CPdraggableDotsMobTransFromParameters.setData(pos=numpy.array([[1000000000,1000000000]])) ###Note: THIS IS NOT GOOD PRACTICE
        self.PlotWidgetIn_pageCP.addItem(self.CPdraggableDotsMobTransFromParameters)
        for clicked in Clicks:
            clicked.clear()
        
      

##############
############## 
#############################################        

    def effectOf_pushButtonCPSGCommon(self):
        #### PERSONAL: should I move the bulk of operations to Steiner_grids_CP.py?
        #I think doing so would create a mess because there would be more dependencies
        try:
            P = extendedValue(self.lineEditCPSGComplexNumber1.text())
            Q = extendedValue(self.lineEditCPSGComplexNumber2.text())
            n = int(self.spinBoxCPSGCommon.cleanText())
            if isooInArgs(P,Q) == False:
                centersAndRadii = Steiner_grids_CP.commonCircles().common_circlesFunction(
                        P, Q, n )
                theta = numpy.linspace(0,1,101)
                for triple in centersAndRadii: ### triple comes in the format [(x,y),r]
                    x_coord = (triple[0])[0] + (triple[1])*numpy.cos(theta*2*numpy.pi)
                    y_coord = (triple[0])[1] + (triple[1])*numpy.sin(theta*2*numpy.pi)
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen=self.blackPenWidth2)
            else:
                finitePoint = removeooFromArgs(P,Q)[0]
                theta = numpy.linspace(0,360,n)
                for t in theta: ### triple comes in the format [[R.real,R.imag],P], where R and P are (finite) complex numbers. PERSONAL NOTE: This is NOT elegant!!!!!!
                    line = pg.InfiniteLine(pos = [finitePoint.real,finitePoint.imag], angle = t, pen=self.blackPenWidth2)
                    self.PlotWidgetIn_pageCP.addItem(line)
        except: #### Implement a pop-up window???
            pass

##############
############## 
#############################################
            
    def effectOf_pushButtonCPSGApollonius(self): ## WARNING(?): THE EXACT SAME CODE APPEARS TWICE, EXCEPT FOR importing.Apollonius_e_circles1 AND importing.Apollonius_e_circles2
        try:
            P = extendedValue(self.lineEditCPSGComplexNumber1.text())
            Q = extendedValue(self.lineEditCPSGComplexNumber2.text())
            n = int(self.spinBoxCPSGCommon.cleanText())
            if isooInArgs(P,Q) == False:
                centersAndRadii = Steiner_grids_CP.Apollonius().Apollonius_e_circles1(
                        P,Q,n)
                theta = numpy.linspace(0,1,101)
                for triple in centersAndRadii: ### triple comes in the format [(x,y),r]
                    x_coord = (triple[0])[0] + (triple[1])*numpy.cos(theta*2*numpy.pi)
                    y_coord = (triple[0])[1] + (triple[1])*numpy.sin(theta*2*numpy.pi)
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen =self.blackPenWidth2)
                centersAndRadii = Steiner_grids_CP.Apollonius().Apollonius_e_circles2(
                        P,Q,n)
                theta = numpy.linspace(0,1,101)
                for triple in centersAndRadii: ### triple comes in the format [(x,y),r]
                    x_coord = (triple[0])[0] + (triple[1])*numpy.cos(theta*2*numpy.pi)
                    y_coord = (triple[0])[1] + (triple[1])*numpy.sin(theta*2*numpy.pi)
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen=self.blackPenWidth2)
            else:
                finitePoint = removeooFromArgs(P,Q)[0]
                theta = numpy.linspace(0,1,101)
                for t in range(1,(n)**2+1,n):
                    x_coord = finitePoint.real + t*numpy.cos(theta*2*numpy.pi)
                    y_coord = finitePoint.imag + t*numpy.sin(theta*2*numpy.pi)
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen=self.blackPenWidth2)
        except:
            pass

##############
############## 
#############################################
                
    def effectOf_pushButtonCPSGSteiner(self):
        self.effectOf_pushButtonCPSGCommon()
        self.effectOf_pushButtonCPSGApollonius()
        
        
##############
############## 
#############################################          
    
    def effectOf_horizontalSliderCPSGLoxodromesAngleTheta(self):
        try:
            self.PlotWidgetIn_pageCP.clear()
            P = extendedValue(self.lineEditCPSGComplexNumber1.text())
            Q = extendedValue(self.lineEditCPSGComplexNumber2.text())
            n = int(self.spinBoxCPSGCommon.cleanText())
            theta = int(self.horizontalSliderCPSGLoxodromesAngleTheta.value())*2*numpy.pi/360
            coordList = Steiner_grids_CP.loxodromes().loxCurve(P,Q,theta,n)
            for coord in coordList:
                self.PlotWidgetIn_pageCP.plot(coord[0],coord[1],pen=self.blackPenWidth2)
        except:
            pass
        
#############################################
##############
############## MOBIUS TRANSFORMATIONS FROM PARAMETERS
            
        
        

    def CPMTAlphaParameter(self,pt):
        self.lineEditCPMTMobFromParam.setText(str(pt[0]+pt[1]*(1j)))
        self.labelCPMTxNumber.setText("<span style='font-size: 12pt'><span style='color: black'>x=%0.100f" % (pt[0]))
        self.labelCPMTyNumber.setText("<span style='font-size: 12pt'><span style='color: black'>y=%0.100f" % (pt[1]))   
        self.labelCPMTNormNumber.setText("<span style='font-size: 12pt'><span style='color: black'>Norm=%0.100f" % (numpy.sqrt(((pt[0])**2) + ((pt[1])**2)))) 



    def CPMTMobFromParamFixedPoints(self,ev):
        if self.checkBoxCPMTMobFromParam.isChecked() == True and self.stackedWidgetIn_pageCP.currentIndex() == 1:
            global twoClicks
            global auxStorage
            x = self.PlotWidgetIn_pageCP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageCP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            twoClicks.append([x,y])
            if len(twoClicks)==1:
                points = numpy.array([[twoClicks[0][0],twoClicks[0][1]]],dtype=float)
                #initialPoint = pg.GraphItem(pos=[[twoClicks[0][0],twoClicks[0][1]]])
                self.CPdraggableDotsMobTransFromParameters.setData(pos=points,  pxMode=True)
                z0 = twoClicks[-1][0]+twoClicks[-1][1]*(1j)
                auxStorage.clear()
                auxStorage.append(z0)
            else:
                #print(twoClicks)
                points = numpy.array([[twoClicks[k][0],twoClicks[k][1]] for k in range(len(twoClicks))],dtype=float)
                self.CPdraggableDotsMobTransFromParameters.setData(pos=points,  pxMode=True)
                #self.PlotWidgetIn_pageUHP.addItem(finalPoint)
                z0 = twoClicks[-2][0]+twoClicks[-2][1]*(1j)
                z1 = twoClicks[-1][0]+twoClicks[-1][1]*(1j)
#                self.lineEditUHPGMComplexNumber1.setText(str(z0))
#                self.lineEditUHPGMComplexNumber2.setText(str(z0))
                twoClicks.clear()
                auxStorage.clear()
                auxStorage.append(z0)
                auxStorage.append(z1)
                







            def effectOf_pushButtonCPMTMobFromParam():
                if self.checkBoxCPMTMobFromParam.isChecked() == True and self.stackedWidgetIn_pageCP.currentIndex() == 1:
                    complexalpha = numpy.complex(str(self.lineEditCPMTMobFromParam.text()))
                    A = Mobius_CP.MobiusFromParameters().MobMatrixFromParams(auxStorage,complexalpha)
                    a = A[0,0]
                    b = A[0,1]
                    c = A[1,0]
                    d = A[1,1]
                    self.lineEditCPMTOrbitsComplexNumberalpha.setText(str(a))
                    self.lineEditCPMTOrbitsComplexNumberbeta.setText(str(b))
                    self.lineEditCPMTOrbitsComplexNumbergamma.setText(str(c))
                    self.lineEditCPMTOrbitsComplexNumberdelta.setText(str(d))
                        
            self.pushButtonCPMTMobFromParam.clicked.connect(effectOf_pushButtonCPMTMobFromParam)                    
                
            
            def effectOf_pushButtonCPMTAnimOrbitSinglePoint():#1,100,1j,1 is a nice loxodromic Mobius transformation for examples
                complexalpha = numpy.complex(str(self.lineEditCPMTMobFromParam.text()))
                a = self.lineEditCPMTOrbitsComplexNumberalpha.text()
                b = self.lineEditCPMTOrbitsComplexNumberbeta.text()
                c = self.lineEditCPMTOrbitsComplexNumbergamma.text()
                d = self.lineEditCPMTOrbitsComplexNumberdelta.text()
                z_0 = extendedValue(self.lineEditCPMTOrbitsComplexNumberz_0.text())
                transformation = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(a,b,c,d)
                numberOfIterations = int(self.spinBoxCPMTOrbits.cleanText())
                OrbitAllPts = Mobius_CP.MobiusAssocToMatrix().MobTransOrbit(a,b,c,d,numberOfIterations)(z_0)[0]
                OrbitFinitePts = Mobius_CP.MobiusAssocToMatrix().MobTransOrbit(a,b,c,d,numberOfIterations)(z_0)[1]
                PartialOrbit = dD.draggableDot()
                CurrentPoint = dD.draggableDot()
                #Dots.Dot.moved.connect(CPMTMobiusOrbitDrag)
                self.PlotWidgetIn_pageCP.addItem(PartialOrbit)
                self.PlotWidgetIn_pageCP.addItem(CurrentPoint)
                self.labelCPMTTypeOfMobius.setText(str(Mobius_CP.MobiusAssocToMatrix().isParEllHypLox(a,b,c,d)[0]))
                
                
        
                if z_0 != oo:
                    self.radioButtonCPoo.setChecked(False)
                    points = numpy.array([[z_0.real,z_0.imag]],dtype=float)
                    PartialOrbit.setData(pos=points,  pxMode=True)
                else:
                    self.radioButtonCPoo.setChecked(True)
                    
                if self.checkBoxCPMTInvariantCurve.isChecked() == True:
                        curves = Mobius_CP.MobiusAssocToMatrix().invariantCurveThroughPt(a,b,c,d,z_0)
                        drawings = []
                        if len(curves)>0:
                            for curve in curves:
                                drawing = pg.PlotCurveItem(curve[0],curve[1],pen=self.blackPenWidth2)
                                drawings.append(drawing)
                                self.PlotWidgetIn_pageCP.addItem(drawing)
                
                OrbitFinitePtsForCircles = [OrbitFinitePts[numpy.sign(numberOfIterations)*i] for i in range(abs(numberOfIterations)) if numpy.sign(numberOfIterations)*i in OrbitFinitePts]
                # ApolloniusCircles = []
                # CommonCircles = []        
                # for i in range(len(OrbitFinitePtsForCircles)):
                #     if len(auxStorage)==1:
                #         z_i = OrbitFinitePtsForCircles[numpy.sign(numberOfIterations)*i]
                #         auxPt1, auxPt2, auxPt3 = z_i, transformation(z_i), transformation(transformation(z_i))
                #         AppolloniuseCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxPt1, auxPt2, auxPt3)
                #         AppolloniuseCenter = AppolloniuseCenterAndRadius[0]
                #         AppolloniuseRadius = AppolloniuseCenterAndRadius[1]
                #         t = numpy.linspace(0, 2*numpy.pi,100)
                #         Appolloniusx_coord, Appolloniusy_coord = AppolloniuseCenter[0]+AppolloniuseRadius*numpy.cos(t), AppolloniuseCenter[1]+AppolloniuseRadius*numpy.sin(t)
                #         ApolloniusCircleThroughz_i = pg.PlotCurveItem(Appolloniusx_coord,Appolloniusy_coord,pen=self.redPenWidth2)
                #         ApolloniusCircles.append(ApolloniusCircleThroughz_i)
                #         #self.PlotWidgetIn_pageCP.addItem(ApolloniusCircleThroughz_i)
                #         auxMatrix = Mobius_CP.MobiusFromParameters().MobMatrixFromParams(auxStorage,(1j)*complexalpha)
                #         auxTrans = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(auxMatrix[0,0],auxMatrix[0,1],auxMatrix[1,0],auxMatrix[1,1])
                #         CommoneCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxStorage[0],z_i,auxTrans(z_i))
                #         CommoneCenter = CommoneCenterAndRadius[0]
                #         CommoneRadius = CommoneCenterAndRadius[1]
                #         t = numpy.linspace(0, 2*numpy.pi,100)
                #         Commonx_coord, Commony_coord = CommoneCenter[0]+CommoneRadius*numpy.cos(t), CommoneCenter[1]+CommoneRadius*numpy.sin(t)
                #         commonCircleThroughz_i = pg.PlotCurveItem(Commonx_coord,Commony_coord,pen=self.bluePenWidth2)
                #         CommonCircles.append(commonCircleThroughz_i)
                #     if len(auxStorage)==2:
                #         z_i = OrbitFinitePtsForCircles[numpy.sign(numberOfIterations)*i]
                #         conjMatrix = Mobius_CP.MobiusTransitivity().MobiusMatrix0oo1Toz1z2z3(auxStorage[0],auxStorage[1],z_i) # THIS ASSUMES THAT z_i IS NOT A FIXED POINT!!!!!
                #         conjTrans = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(conjMatrix[0,0],conjMatrix[0,1],conjMatrix[1,0],conjMatrix[1,1])
                #         auxPt1, auxPt2, auxPt3 = z_i, conjTrans(1j), conjTrans(-1)
                #         AppolloniuseCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxPt1, auxPt2, auxPt3)
                #         AppolloniuseCenter = AppolloniuseCenterAndRadius[0]
                #         AppolloniuseRadius = AppolloniuseCenterAndRadius[1]
                #         t = numpy.linspace(0, 2*numpy.pi,100)
                #         Appolloniusx_coord, Appolloniusy_coord = AppolloniuseCenter[0]+AppolloniuseRadius*numpy.cos(t), AppolloniuseCenter[1]+AppolloniuseRadius*numpy.sin(t)
                #         ApolloniusCircleThroughz_i = pg.PlotCurveItem(Appolloniusx_coord,Appolloniusy_coord,pen=self.redPenWidth2)
                #         ApolloniusCircles.append(ApolloniusCircleThroughz_i)
                #         #self.PlotWidgetIn_pageCP.addItem(ApolloniusCircleThroughz_i)
                #         CommoneCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxStorage[0],auxStorage[1],z_i)
                #         CommoneCenter = CommoneCenterAndRadius[0]
                #         CommoneRadius = CommoneCenterAndRadius[1]
                #         t = numpy.linspace(0, 2*numpy.pi,100)
                #         Commonx_coord, Commony_coord = CommoneCenter[0]+CommoneRadius*numpy.cos(t), CommoneCenter[1]+CommoneRadius*numpy.sin(t)
                #         commonCircleThroughz_i = pg.PlotCurveItem(Commonx_coord,Commony_coord,pen=self.bluePenWidth2)
                #         CommonCircles.append(commonCircleThroughz_i)
                #         #self.PlotWidgetIn_pageCP.addItem(commonCircleThroughz_i)
                
                # conjMatrix = Mobius_CP.MobiusTransitivity().MobiusMatrix0oo1Toz1z2z3(auxStorage[0],auxStorage[1],z_0) # THIS ASSUMES THAT z_0 IS NOT A FIXED POINT!!!!!
                # conjTrans = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(conjMatrix[0,0],conjMatrix[0,1],conjMatrix[1,0],conjMatrix[1,1])
                # auxPt1, auxPt2, auxPt3 = z_0, conjTrans(1j), conjTrans(-1)
                # eCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxPt1, auxPt2, auxPt3)
                # eCenter = eCenterAndRadius[0]
                # eRadius = eCenterAndRadius[1]
                # t = numpy.linspace(0, 2*numpy.pi,100)
                # x_coord, y_coord = eCenter[0]+eRadius*numpy.cos(t), eCenter[1]+eRadius*numpy.sin(t)
                # ApolloniusCircleThroughCurrent_z = pg.PlotCurveItem(pen=self.redPenWidth2)
                # self.PlotWidgetIn_pageCP.addItem(ApolloniusCircleThroughCurrent_z)
                # #ApolloniusCircleThroughCurrent_z.setData(x_coord,y_coord)
                # eCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxStorage[0],auxStorage[1],z_0)
                # eCenter = eCenterAndRadius[0]
                # eRadius = eCenterAndRadius[1]
                # t = numpy.linspace(0, 2*numpy.pi,100)
                # x_coord, y_coord = eCenter[0]+eRadius*numpy.cos(t), eCenter[1]+eRadius*numpy.sin(t)
                # commonCircleThroughCurrent_z = pg.PlotCurveItem(pen=self.bluePenWidth2)
                # self.PlotWidgetIn_pageCP.addItem(commonCircleThroughCurrent_z)
                # #commonCircleThroughCurrent_z.setData(x_coord,y_coord)
                               

                
                k=0
                def update():
                    self.radioButtonCPoo.setChecked(False)
                    nonlocal k
                    points = numpy.array([[OrbitFinitePts[numpy.sign(numberOfIterations)*i].real,OrbitFinitePts[numpy.sign(numberOfIterations)*i].imag] for i in range(k+1) if numpy.sign(numberOfIterations)*i in OrbitFinitePts],dtype=float)
                    if len(points) > 0:
                        PartialOrbit.setData(pos=points, pxMode=True)                            
  
                    current_z = OrbitAllPts[numpy.sign(numberOfIterations)*(k)%numberOfIterations]
                    if current_z !=oo:
                        current = numpy.array([[current_z.real,current_z.imag]],dtype=float)
                        CurrentPoint.setData(pos=current, symbolBrush=(217,83,25), symbolPen='r',  pxMode=True)
                        # if len(auxStorage)==1:
                        #     pass
                        # if len(auxStorage)==2:
                        #     conjMatrix = Mobius_CP.MobiusTransitivity().MobiusMatrix0oo1Toz1z2z3(auxStorage[0],auxStorage[1],current_z) # THIS ASSUMES THAT current_z IS NOT A FIXED POINT!!!!!
                        #     conjTrans = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(conjMatrix[0,0],conjMatrix[0,1],conjMatrix[1,0],conjMatrix[1,1])
                        #     auxPt1, auxPt2, auxPt3 = current_z, conjTrans(1j), conjTrans(-1)
                        #     eCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxPt1, auxPt2, auxPt3)
                        #     eCenter = eCenterAndRadius[0]
                        #     eRadius = eCenterAndRadius[1]
                        #     t = numpy.linspace(0, 2*numpy.pi,100)
                        #     x_coord, y_coord = eCenter[0]+eRadius*numpy.cos(t), eCenter[1]+eRadius*numpy.sin(t)
                        #     ApolloniusCircleThroughCurrent_z.setData(x_coord,y_coord)
                        #     eCenterAndRadius = extended_complex_plane_CP.numpyExtendedComplexPlane().e_circumcenter_and_radius(auxStorage[0],auxStorage[1],current_z)
                        #     eCenter = eCenterAndRadius[0]
                        #     eRadius = eCenterAndRadius[1]
                        #     t = numpy.linspace(0, 2*numpy.pi,100)
                        #     x_coord, y_coord = eCenter[0]+eRadius*numpy.cos(t), eCenter[1]+eRadius*numpy.sin(t)
                        #     commonCircleThroughCurrent_z.setData(x_coord,y_coord)
                            
                        # for i in range(k+1):
                        #     self.PlotWidgetIn_pageCP.addItem(ApolloniusCircles[i])
                        #     self.PlotWidgetIn_pageCP.addItem(CommonCircles[i])
                            
                    else:
                        self.radioButtonCPoo.setChecked(True)
                        CurrentPoint.setData(symbolBrush=(0,0,200))
                    if k+1 == abs(numberOfIterations):
                        for i in range(k+1):
                            self.PlotWidgetIn_pageCP.clear()
                            self.PlotWidgetIn_pageCP.clear()
                        self.PlotWidgetIn_pageCP.addItem(PartialOrbit)
                        self.PlotWidgetIn_pageCP.addItem(CurrentPoint)
                    k = (k+1)%abs(numberOfIterations)
        
                if self.timer:
                    self.timer.stop()
                    self.timer.deleteLater()
                self.timer = QtCore.QTimer(self)
                self.timer.timeout.connect(update)
                self.timer.start(250)
                
            self.pushButtonCPMTAnimOrbitSinglePoint.clicked.connect(effectOf_pushButtonCPMTAnimOrbitSinglePoint) 
        






########################



    def effectOf_pushButtonCPMTFixedPoints(self):
        a = self.lineEditCPMTOrbitsComplexNumberalpha.text()
        b = self.lineEditCPMTOrbitsComplexNumberbeta.text()
        c = self.lineEditCPMTOrbitsComplexNumbergamma.text()
        d = self.lineEditCPMTOrbitsComplexNumberdelta.text()
        FixedPts = Mobius_CP.MobiusAssocToMatrix().fixedPoints(a,b,c,d)
        FixedPtsPos = numpy.array([[FixedPts[i].real,FixedPts[i].imag] for i in range(2) if FixedPts[i]!=oo],dtype=float)
        if len(FixedPtsPos) > 0:
            FixedPtsDots = pg.ScatterPlotItem(pos=FixedPtsPos)
            self.PlotWidgetIn_pageCP.addItem(FixedPtsDots)
        self.radioButtonCPoo.setChecked(False)
        if oo in FixedPts:
            self.radioButtonCPoo.setChecked(True)
            

        
        
        
    def effectOf_pushButtonCPMTStaticOrbitSinglePoint(self): ## PERSONAL NOTE: Check the code for curves...
        #1+100j, 200, 2j, 1+100j
        try:
            a = self.lineEditCPMTOrbitsComplexNumberalpha.text()
            b = self.lineEditCPMTOrbitsComplexNumberbeta.text()
            c = self.lineEditCPMTOrbitsComplexNumbergamma.text()
            d = self.lineEditCPMTOrbitsComplexNumberdelta.text()
            z_0 = extendedValue(self.lineEditCPMTOrbitsComplexNumberz_0.text())
            numberOfIterations = int(self.spinBoxCPMTOrbits.cleanText())
            OrbitAllPts = Mobius_CP.MobiusAssocToMatrix().MobTransOrbit(a,b,c,d,numberOfIterations)(z_0)[0]
            OrbitFinitePts = Mobius_CP.MobiusAssocToMatrix().MobTransOrbit(a,b,c,d,numberOfIterations)(z_0)[1]
            Dots = dD.draggableDot()
            self.PlotWidgetIn_pageCP.addItem(Dots)
            self.labelCPMTTypeOfMobius.setText(str(Mobius_CP.MobiusAssocToMatrix().isParEllHypLox(a,b,c,d)[0]))
            
            drawings = []
            if self.checkBoxCPMTInvariantCurve.isChecked() == True:
                curves = Mobius_CP.MobiusAssocToMatrix().invariantCurveThroughPt(a,b,c,d,z_0)
                
                if len(curves)>0:
                    for curve in curves:
                        drawing = pg.PlotCurveItem(curve[0],curve[1],pen=self.blackPenWidth2)
                        drawings.append(drawing)
                        self.PlotWidgetIn_pageCP.addItem(drawing)
                    
            points = numpy.array([[OrbitFinitePts[numpy.sign(numberOfIterations)*i].real,OrbitFinitePts[numpy.sign(numberOfIterations)*i].imag] for i in range(abs(numberOfIterations)+1) if numpy.sign(numberOfIterations)*i in OrbitFinitePts],dtype=float)
            if len(points) > 0:
                Dots.setData(pos=points,  pxMode=True)
            self.radioButtonCPoo.setChecked(False)
            if len(OrbitFinitePts) < len(OrbitAllPts):
                self.radioButtonCPoo.setChecked(True)
                
            def CPMTMobiusOrbitDrag(pt,ind):
                nonlocal drawings
                try:
                    P = pt[0]+pt[1]*(1j)
                    k = numpy.sign(numberOfIterations)*ind
                    z_0 = Mobius_CP.MobiusAssocToMatrix().MobTrans_nthPowerEvalAtConcretePoint(a,b,c,d,-k)(P)
                    OrbitAllPts = Mobius_CP.MobiusAssocToMatrix().MobTransOrbit(a,b,c,d,numberOfIterations)(z_0)[0]
                    OrbitFinitePts = Mobius_CP.MobiusAssocToMatrix().MobTransOrbit(a,b,c,d,numberOfIterations)(z_0)[1]
                    
                    if self.checkBoxCPMTInvariantCurve.isChecked() == True:
                        currentCurves = Mobius_CP.MobiusAssocToMatrix().invariantCurveThroughPt(a,b,c,d,z_0)
                        
                        if len(drawings)>0 and len(currentCurves)>0:
                            for i in range(0,len(drawings),1):
                                drawings[i].setData(currentCurves[i][0],currentCurves[i][1])
                    
                    
                    points = numpy.array([[OrbitFinitePts[numpy.sign(numberOfIterations)*i].real,OrbitFinitePts[numpy.sign(numberOfIterations)*i].imag] for i in range(abs(numberOfIterations)+1) if numpy.sign(numberOfIterations)*i in OrbitFinitePts],dtype=float)
                    if len(points) > 0:
                        Dots.setData(pos=points,  pxMode=True)
                    if len(OrbitFinitePts) < len(OrbitAllPts):
                        self.radioButtonCPoo.setChecked(True)
                    if len(OrbitFinitePts) == len(OrbitAllPts):
                        self.radioButtonCPoo.setChecked(False)
                        
                    self.lineEditCPMTOrbitsComplexNumberz_0.setText(str(z_0))
                except:
                    pass
            Dots.Dot.moved.connect(CPMTMobiusOrbitDrag)
        
        except:
            pass
##############
############## 
#############################################
 
        
    def effectOf_pushButtonCPMTOrbitsRandomCircle(self):
        alpha = self.lineEditCPMTOrbitsComplexNumberalpha.text() 
        beta = self.lineEditCPMTOrbitsComplexNumberbeta.text()
        gamma = self.lineEditCPMTOrbitsComplexNumbergamma.text()
        delta = self.lineEditCPMTOrbitsComplexNumberdelta.text()
        MobiusTrans = Mobius_CP.MobiusAssocToMatrix(alpha,beta,gamma,delta)
        P = numpy.complex(numpy.random.random(), numpy.random.random())
        Q = numpy.complex(numpy.random.random(), numpy.random.random())
        R = numpy.complex(numpy.random.random(), numpy.random.random())
        numberOfPointsInOrbit = int(self.spinBoxCPMTOrbits.cleanText())
        OrbitP = MobiusTrans.Mob_trans_iterable(P,numberOfPointsInOrbit)
        OrbitQ = MobiusTrans.Mob_trans_iterable(Q,numberOfPointsInOrbit)
        OrbitR = MobiusTrans.Mob_trans_iterable(R,numberOfPointsInOrbit)
        centersAndRadii = [e_circumcenter_and_radius(OrbitP[k],OrbitQ[k],OrbitR[k]) for k in range(0,numberOfPointsInOrbit,1)]
        t = numpy.linspace(0, 2*numpy.pi,500)
#        for k in range(0,numberOfPointsInOrbit,1):
#            x_coord = ((centersAndRadii[k])[0])[0] + (centersAndRadii[k])[1]*numpy.cos(t)
#            y_coord = ((centersAndRadii[k])[0])[1] + (centersAndRadii[k])[1]*numpy.sin(t)
#            self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen='m')
        x_coord = ((centersAndRadii[0])[0])[0] + (centersAndRadii[0])[1]*numpy.cos(t)
        y_coord = ((centersAndRadii[0])[0])[1] + (centersAndRadii[0])[1]*numpy.sin(t)
        self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen=self.blackPenWidth2)
        k = 0
        def update():
            nonlocal k
            X_current = ((centersAndRadii[k])[0])[0] + (centersAndRadii[k])[1]*numpy.cos(t)
            Y_current = ((centersAndRadii[k])[0])[1] + (centersAndRadii[k])[1]*numpy.sin(t)
            #C = pyqtgraph.hsvColor(time.time()/5%1,alpha=.5)
            #pen=pyqtgraph.mkPen(color=C,width=10)
            self.PlotWidgetIn_pageCP.plot(X_current,Y_current,pen='r',clear=False)
            X_next = ((centersAndRadii[(k+1)%numberOfPointsInOrbit])[0])[0] + (centersAndRadii[(k+1)%numberOfPointsInOrbit])[1]*numpy.cos(t)
            Y_next = ((centersAndRadii[(k+1)%numberOfPointsInOrbit])[0])[1] + (centersAndRadii[(k+1)%numberOfPointsInOrbit])[1]*numpy.sin(t)
            #C = pyqtgraph.hsvColor(time.time()/5%1,alpha=.5)
            #pen=pyqtgraph.mkPen(color=C,width=10)
            self.PlotWidgetIn_pageCP.plot(X_next,Y_next,pen=self.blackPenWidth2,clear=False)
            k = (k+1)%numberOfPointsInOrbit
        #    QtCore.QTimer.singleShot(1000, update)
        #update()
        
        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(update)
        self.timer.start(100)

##############
############## 
#############################################        
            
    def effectOf_pushButtonCPMTOrbitsSteinerGrid(self):
        nIterations = int(self.spinBoxCPMTOrbits.cleanText())
        alpha = self.lineEditCPMTOrbitsComplexNumberalpha.text() 
        beta = self.lineEditCPMTOrbitsComplexNumberbeta.text()
        gamma = self.lineEditCPMTOrbitsComplexNumbergamma.text()
        delta = self.lineEditCPMTOrbitsComplexNumberdelta.text()
        MobiusTrans = Mobius_CP.MobiusAssocToMatrix(alpha,beta,gamma,delta)
        fixedPoints = MobiusTrans.fixedPoints()
        P = fixedPoints[0]
        Q = fixedPoints[1]
        
        
        if MobiusTrans.isParEllHypLox() == 'PARABOLIC':
            ThreePoints = MobiusTrans.frameOfParabolic()[0]
            Orbit1 = MobiusTrans.Mob_trans_iterable(ThreePoints[0],nIterations)
            Orbit2 = MobiusTrans.Mob_trans_iterable(ThreePoints[1],nIterations)
            Orbit3 = MobiusTrans.Mob_trans_iterable(ThreePoints[2],nIterations)
            
            for p in ThreePoints:
                self.plottingPointInExtendedPlane(p,'y','y')
            
            k=0
            def update():
                nonlocal k
                previous_Pt1 = Orbit1[k]
                previous_Pt2 = Orbit2[k]
                previous_Pt3 = Orbit3[k]
                #self.PlottingCircleOrLineThrough3Points(previous_Pt1,previous_Pt2,previous_Pt3,'r')
                #self.PlottingCircleOrLineThrough3Points(P,previous_Pt2,Q,'m')
                previous_points = pg.ScatterPlotItem([previous_Pt1.real,previous_Pt2.real,previous_Pt3.real],[previous_Pt1.imag,previous_Pt2.imag,previous_Pt3.imag],pen='m')
                self.PlotWidgetIn_pageCP.addItem(previous_points)
                current_Pt1 = Orbit1[(k+1)%nIterations]
                current_Pt2 = Orbit2[(k+1)%nIterations]
                current_Pt3 = Orbit3[(k+1)%nIterations]
                #self.PlottingCircleOrLineThrough3Points(current_Pt1,current_Pt2,current_Pt3,'k')
                #self.PlottingCircleOrLineThrough3Points(P,current_Pt2,Q,'y')
                current_points = pg.ScatterPlotItem([current_Pt1.real,current_Pt2.real,current_Pt3.real],[current_Pt1.imag,current_Pt2.imag,current_Pt3.imag],pen='y')
                self.PlotWidgetIn_pageCP.addItem(current_points)
                k = (k+1)%nIterations
            
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(update)
            self.timer.start(500)
            
        

        
        
        
        
        if isooInArgs(P,Q) == False and P != Q:
            ThreePointsOnMediatrix = Steiner_grids_CP.commonCircles().pointsOnMediatrix(P,Q,3)
            Pt1OnMdx = ThreePointsOnMediatrix[0] 
            Pt2OnMdx = ThreePointsOnMediatrix[1]
            Pt3OnMdx = ThreePointsOnMediatrix[2]
            Orbit1 = MobiusTrans.Mob_trans_iterable(Pt1OnMdx,nIterations)
            Orbit2 = MobiusTrans.Mob_trans_iterable(Pt2OnMdx,nIterations)
            Orbit3 = MobiusTrans.Mob_trans_iterable(Pt3OnMdx,nIterations)
            
            self.PlottingCircleOrLineThrough3Points(Pt1OnMdx,Pt2OnMdx,Pt3OnMdx,'k')
            self.PlottingCircleOrLineThrough3Points(P,Pt2OnMdx,Q,'y')
            point = pg.ScatterPlotItem([Pt2OnMdx.real],[Pt2OnMdx.imag],pen='y')
            self.PlotWidgetIn_pageCP.addItem(point)
            
            k=0
            def update():
                nonlocal k
                previous_Pt1 = Orbit1[k]
                previous_Pt2 = Orbit2[k]
                previous_Pt3 = Orbit3[k]
                self.PlottingCircleOrLineThrough3Points(previous_Pt1,previous_Pt2,previous_Pt3,'r')
                self.PlottingCircleOrLineThrough3Points(P,previous_Pt2,Q,'m')
                previous_point = pg.ScatterPlotItem([previous_Pt2.real],[previous_Pt2.imag],pen='m')
                self.PlotWidgetIn_pageCP.addItem(previous_point)
                current_Pt1 = Orbit1[(k+1)%nIterations]
                current_Pt2 = Orbit2[(k+1)%nIterations]
                current_Pt3 = Orbit3[(k+1)%nIterations]
                self.PlottingCircleOrLineThrough3Points(current_Pt1,current_Pt2,current_Pt3,'k')
                self.PlottingCircleOrLineThrough3Points(P,current_Pt2,Q,'y')
                current_point = pg.ScatterPlotItem([current_Pt2.real],[current_Pt2.imag],pen='y')
                self.PlotWidgetIn_pageCP.addItem(current_point)
                k = (k+1)%nIterations
            #    QtCore.QTimer.singleShot(1000, update)
            #update()
            
    #        timer = QtCore.QTimer(self)
    #        timer.timeout.connect(update)
    #        timer.start(250)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(update)
            self.timer.start(500)
            
        if isooInArgs(P,Q) == True and P != Q:
            finitePoint = removeooFromArgs(P,Q)[0]
            Orbit1 = MobiusTrans.Mob_trans_iterable(finitePoint+1,nIterations)
            Orbit2 = MobiusTrans.Mob_trans_iterable(finitePoint+1j,nIterations)
            Orbit3 = MobiusTrans.Mob_trans_iterable(finitePoint-1,nIterations)
            
            self.PlottingCircleOrLineThrough3Points(finitePoint+1,finitePoint+1j,finitePoint-1,'k')
            self.PlottingCircleOrLineThrough3Points(P,finitePoint+1j,Q,'y')
            point = pg.ScatterPlotItem([(finitePoint+1j).real],[(finitePoint+1j).imag])
            self.PlotWidgetIn_pageCP.addItem(point)
            
            k=0
            def update():
                nonlocal k
                previous_Pt1 = Orbit1[k]
                previous_Pt2 = Orbit2[k]
                previous_Pt3 = Orbit3[k]
                self.PlottingCircleOrLineThrough3Points(previous_Pt1,previous_Pt2,previous_Pt3,'r')
                self.PlottingCircleOrLineThrough3Points(P,previous_Pt2,Q,'m')
                previous_point = pg.ScatterPlotItem([previous_Pt2.real],[previous_Pt2.imag],pen='m')
                self.PlotWidgetIn_pageCP.addItem(previous_point)
                current_Pt1 = Orbit1[(k+1)%nIterations]
                current_Pt2 = Orbit2[(k+1)%nIterations]
                current_Pt3 = Orbit3[(k+1)%nIterations]
                self.PlottingCircleOrLineThrough3Points(current_Pt1,current_Pt2,current_Pt3,'k')
                self.PlottingCircleOrLineThrough3Points(P,current_Pt2,Q,'y')
                current_point = pg.ScatterPlotItem([current_Pt2.real],[current_Pt2.imag],pen='y')
                self.PlotWidgetIn_pageCP.addItem(current_point)
                k = (k+1)%nIterations
            #    QtCore.QTimer.singleShot(1000, update)
            #update()
            
    #        timer = QtCore.QTimer(self)
    #        timer.timeout.connect(update)
    #        timer.start(250)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(update)
            self.timer.start(500)
                
            
            
            

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
#################################
#################################
#################################
### HYPERBOLIC PLANE
### UPPER HALF PLANE --- UHP


    def effectOf_pushButtonUHPClearCanvas(self):
        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
            self.timer = None
        self.PlotWidgetIn_pageUHP.clear()
        self.UHPdraggableDotsStaticGeodSegs.setData(pos=numpy.array([[0,-100]]))
        self.UHPdraggableDotsConvexHull.setData(pos=numpy.array([[0,-100]]))
        self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDotsStaticGeodSegs)
        self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDotsConvexHull)
        self.PlotWidgetIn_pageUHP.addItem(self.boundingLineUHP)
        self.boundingLineUHP.setPen(pg.mkPen('k', width=self.widthBoundingLineUHP))
        for clicked in Clicks:
            clicked.clear()
            

        



#    
#    def effectOf_UHPsigMouseClicked_Coords(self,ev): ## ev is the clicked point
#        global twoClicks
#        x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
#        y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
#        while len(twoClicks) < 3:  
#            twoClicks.append([x,y])
#        if len(twoClicks) == 3:  
#            del twoClicks[0]
#        print(twoClicks)
#        return twoClicks
#    
#

    def UHPmouseMoved(self,evt):
        pos = evt[0]  ## using signal proxy turns original arguments into a tuple
#        if self.PlotWidgetIn_pageUHP.sceneBoundingRect().contains(pos):
        mousePoint = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(pos)
        #index = int(mousePoint.x())
        if mousePoint.y() >0:
            self.boundingLineUHP.setPen(pg.mkPen('k', width=self.widthBoundingLineUHP))
            self.labelUHPxNumber.setText("<span style='font-size: 12pt'><span style='color: green'>x=%0.100f" % (mousePoint.x()))
            self.labelUHPyNumber.setText("<span style='font-size: 12pt'><span style='color: green'>y=%0.100f" % (mousePoint.y()))
        if mousePoint.y() == 0:
            self.boundingLineUHP.setPen(pg.mkPen('m', width=self.widthBoundingLineUHP))
            self.labelUHPxNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>x=%0.100f" % (mousePoint.x()))
            self.labelUHPyNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>y=%0.100f" % (mousePoint.y()))
        if mousePoint.y() < 0:
            self.boundingLineUHP.setPen(pg.mkPen('r', width=self.widthBoundingLineUHP))
            self.labelUHPxNumber.setText("<span style='font-size: 12pt'><span style='color: red'>x=%0.100f" % (mousePoint.x()))
            self.labelUHPyNumber.setText("<span style='font-size: 12pt'><span style='color: red'>y=%0.100f" % (mousePoint.y()))


    def UHPGMmouseMoved(self,evt):
        pos = evt[0]
        mousePoint = self.PlotWidgetIn_pageUHPGeodesicMotion.plotItem.vb.mapSceneToView(pos)
        self.horLineCrosshairUHPGM.setPos(mousePoint.y())
        self.vertLineCrosshairUHPGM.setPos(mousePoint.x())
        

    
    
    def UHPBCGeodesicSegmentStatic(self,ev):
        if self.radioButtonUHPBCGeodesicSegments.isChecked() == True and self.stackedWidgetIn_pageUHP.currentIndex() == 0:
            global arbManyClicks
            global auxStorage
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y < 0:
                pass
            else:
                arbManyClicks.append([x,y])
                if len(arbManyClicks)==1:
                    points = numpy.array([[arbManyClicks[0][0],arbManyClicks[0][1]]],dtype=float)
                    #initialPoint = pg.GraphItem(pos=[[twoClicks[0][0],twoClicks[0][1]]])
                    self.UHPdraggableDotsStaticGeodSegs.setData(pos=points,  pxMode=True)
                else:
                    #print(twoClicks)
                    points = numpy.array([[arbManyClicks[k][0],arbManyClicks[k][1]] for k in range(len(arbManyClicks))],dtype=float)
                    self.UHPdraggableDotsStaticGeodSegs.setData(pos=points,  pxMode=True)
                    #self.PlotWidgetIn_pageUHP.addItem(finalPoint)
                    P = arbManyClicks[-2][0]+arbManyClicks[-2][1]*(1j)
                    Q = arbManyClicks[-1][0]+arbManyClicks[-1][1]*(1j)
                    self.lineEditUHPGMComplexNumber1.setText(str(P))
                    self.lineEditUHPGMComplexNumber2.setText(str(Q))
                    #print(P,Q)
                    geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                    drawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                    self.PlotWidgetIn_pageUHP.addItem(drawing)
                    auxStorage.append(drawing)
                    #print(arbManyClicks)
#                    print(auxStorage)
                    self.labelUHPBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q))
                    
#        if self.checkBoxUHPEnableClickOnCanvas.isChecked() == False:
#            for clicked in Clicks:
#                clicked.clear()
            #print("as expected")        
#    @QtCore.pyqtSlot(object,int)
    def UHPBCDragGeodesicSegment(self,pt,ind):
        global arbManyClicks
        global auxStorage
        if len(arbManyClicks) < 2 or ind == -1 or pt[1]<0:
            pass
        else:
            P = pt[0]+pt[1]*(1j)
            if ind == 0:
                neighbour = arbManyClicks[1]
                Q = neighbour[0]+neighbour[1]*(1j)
                curve = auxStorage[0]   
                geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
                self.labelUHPBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q))
            if ind == len(arbManyClicks)-1:
                neighbour = arbManyClicks[ind-1]
                Q = neighbour[0]+neighbour[1]*(1j)
                curve = auxStorage[ind-1]
                geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
                self.labelUHPBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q))
            if 0 < ind and ind < len(arbManyClicks)-1:
                neighbour1 = arbManyClicks[ind-1]
                neighbour2 = arbManyClicks[ind+1]
                Q1 =neighbour1[0]+neighbour1[1]*(1j)
                Q2 = neighbour2[0]+neighbour2[1]*(1j)
                curve1 = auxStorage[ind-1]
                curve2 = auxStorage[ind]
                geodesicSegment1 = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q1)
                geodesicSegment2 = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q2)
                x_coord1, y_coord1 = geodesicSegment1.real, geodesicSegment1.imag
                x_coord2, y_coord2 = geodesicSegment2.real, geodesicSegment2.imag
                curve1.setData(x_coord1,y_coord1)
                curve2.setData(x_coord2,y_coord2)
                #self.labelUHPBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q1))
            arbManyClicks.remove(arbManyClicks[ind])
            arbManyClicks.insert(ind,[pt[0],pt[1]])
                




                
                
            
                
            
                
    
    def UHPBCConvexHull(self,ev):
        if self.radioButtonUHPBCConvexHull.isChecked() == True and self.stackedWidgetIn_pageUHP.currentIndex() == 0:
            global arbManyClicks
            global auxStorage
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y < 0:
                pass
            else:
                arbManyClicks.append(x+y*(1j))
                if len(arbManyClicks) == 1:
                    points = numpy.array([[x,y]],dtype=float)
                    self.UHPdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
#                currentPoint = pg.ScatterPlotItem([x],[y])
#                self.PlotWidgetIn_pageUHP.addItem(currentPoint)
                    auxStorage.append(x+y*(1j))
#                    X, Y = arbManyClicks[0].real, arbManyClicks[0].imag
#                    firstPoint = pg.ScatterPlotItem([X],[Y])
#                    self.PlotWidgetIn_pageUHP.addItem(firstPoint)
                elif len(arbManyClicks) == 2:
                    points = numpy.array([[arbManyClicks[k].real,arbManyClicks[k].imag] for k in range(len(arbManyClicks))],dtype=float)
                    self.UHPdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
                    auxStorage.append(x+y*(1j))
#                    X, Y = arbManyClicks[1].real, arbManyClicks[1].imag
#                    secondPoint = pg.ScatterPlotItem([X],[Y])
#                    self.PlotWidgetIn_pageUHP.addItem(secondPoint)
                    geodesicSeg = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(arbManyClicks[0],arbManyClicks[1])
                    x_coord, y_coord = geodesicSeg.real, geodesicSeg.imag
                    drawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                    self.PlotWidgetIn_pageUHP.addItem(drawing)
                    auxStorage2.append(drawing)
                else:
#                    self.PlotWidgetIn_pageUHP.clear()
#                    self.PlotWidgetIn_pageUHP.addItem(self.boundingLineUHP)
#                    self.boundingLineUHP.setPen(pg.mkPen('k', width=self.widthBoundingLineUHP))
#                    self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDotsConvexHullVert)
#                    self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDotsConvexHullNonVert)
#                    for p in arbManyClicks:
#                        X, Y = p.real, p.imag
#                        Point = pg.ScatterPlotItem([X],[Y])
#                        self.PlotWidgetIn_pageUHP.addItem(Point)
                    if UHP_HP.UHPBasics().is_point_in_h_convex_hull(x+y*(1j),auxStorage) == False:
                        auxStorage.append(x+y*(1j))
                    vertices = UHP_HP.UHPBasics().verts_h_polygon_counter_clockwise(auxStorage)
                    points = numpy.array([[arbManyClicks[k].real,arbManyClicks[k].imag] for k in range(len(arbManyClicks))],dtype=float)
                    self.UHPdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
#                    nonVerts = numpy.array([[arbManyClicks[k].real,arbManyClicks[k].imag] for k in range(len(arbManyClicks)) if arbManyClicks[k] not in vertices],dtype=float)
#                    if len(nonVerts) > 0:
#                        self.UHPdraggableDotsConvexHullNonVert.setData(pos=nonVerts, brush='g',  pxMode=True)
                    for theCurves in auxStorage2:
                        self.PlotWidgetIn_pageUHP.removeItem(theCurves)
                    auxStorage2.clear()
                    #numpy.union1d(numpyLeftInterval,numpyRightInterval)
                    geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(vertices[0],vertices[1])
                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag                  
                    for i in range(1,len(vertices)-1,1):
                        geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(vertices[i],vertices[i+1])
                        x_coord = numpy.concatenate((x_coord,geodesicSegment.real))
                        y_coord = numpy.concatenate((y_coord,geodesicSegment.imag))
                    theDrawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                    self.PlotWidgetIn_pageUHP.addItem(theDrawing)
                    auxStorage2.append(theDrawing)
#                    print("v="+str(len(vertices)))
#                    print(len(auxStorage2))
                    auxStorage.clear()
                    auxStorage.extend(vertices)
#        if self.checkBoxUHPEnableClickOnCanvas.isChecked() == False:
#            for clicked in Clicks:
#                clicked.clear()
                
    def UHPBCDragConvexHull(self,pt,ind):
        global arbManyClicks
        global auxStorage
        global auxStorage2
        if len(arbManyClicks) < 2 or ind == -1 or pt[1]<0:
            pass
        else:
            P = pt[0]+pt[1]*(1j)
            if len(arbManyClicks) == 2:
                Q = arbManyClicks[(ind+1)%2]
                curve = auxStorage2[0]   
                geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
                auxStorage.remove(auxStorage[ind])
                auxStorage.insert(ind,P)
                arbManyClicks.remove(arbManyClicks[ind])
                arbManyClicks.insert(ind,P)
            if len(arbManyClicks) > 2:
                arbManyClicks.remove(arbManyClicks[ind])
                arbManyClicks.insert(ind,P)
                vertices = UHP_HP.UHPBasics().verts_h_polygon_counter_clockwise(arbManyClicks)
                geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(vertices[0],vertices[1])
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag                  
                for i in range(1,len(vertices)-1,1):
                    geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(vertices[i],vertices[i+1])
                    x_coord = numpy.concatenate((x_coord,geodesicSegment.real))
                    y_coord = numpy.concatenate((y_coord,geodesicSegment.imag))
                theDrawing = auxStorage2[0]
                theDrawing.setData(x_coord,y_coord)
#                    print("v="+str(len(vertices)))
#                    print(len(auxStorage2))
                auxStorage.clear()
                auxStorage.extend(vertices)




            
        
                        
                
            
            
    


        
            
        
            

        
    def UHPGMGeodesicSegmentAnimated(self,ev): # point-point
        if self.radioButtonUHPGMGeoParamByArcLength.isChecked() == True and self.radioButtonUHPGMPointPoint.isChecked() == True and self.stackedWidgetIn_pageUHP.currentIndex() == 1:
            global twoClicks
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y <= 0:
                pass
            else:
                while len(twoClicks) < 3:  
                    twoClicks.append([x,y])
                if len(twoClicks) == 3:  
                    del twoClicks[0]
                if twoClicks[0] == twoClicks[1]:
                    initialPoint = pg.ScatterPlotItem([twoClicks[0][0]],[twoClicks[0][1]],pen='r',brush='r')
                    self.PlotWidgetIn_pageUHP.addItem(initialPoint)
                    self.lineEditUHPGMComplexNumber1.setText(str(twoClicks[0][0]+twoClicks[0][1]*(1j)))
                else:
                    #print(twoClicks)
                    finalPoint = pg.ScatterPlotItem([twoClicks[1][0]],[twoClicks[1][1]])
                    self.PlotWidgetIn_pageUHP.addItem(finalPoint)
                    P = twoClicks[0][0]+twoClicks[0][1]*(1j)
                    Q = twoClicks[1][0]+twoClicks[1][1]*(1j)
                    self.lineEditUHPGMComplexNumber2.setText(str(Q))
                    #print(P,Q)
                    geodesicParametrization = UHP_HP.UHPGeodesicMotion().UHPGeodesicSegmentParamByArcLength(P,Q)[0]
                    s = UHP_HP.UHPGeodesicMotion().UHPGeodesicSegmentParamByArcLength(P,Q)[1]
                    self.labelUHPGMhdistancenumber.setNum(s)
                    numberOfSteps = 500
                    t = numpy.linspace(0,s,numberOfSteps+1)
                    plottedCurve = self.PlotWidgetIn_pageUHP.plot(pen = self.blackPenWidth2)
                    k=0
                    initialTime = time()
#                    lastTime = time()
#                    speed = None
#                    def update():
#                        nonlocal k, lastTime, speed
#                        if k < len(t)-1:
#                            partialInterval = numpy.linspace(0,t[k+1],1000)
#                            x_coord = geodesicParametrization(partialInterval).real
#                            y_coord = geodesicParametrization(partialInterval).imag
#                            plottedCurve.setData(x_coord,y_coord)
#                            #self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='k')   
#                            now = time()
#                            dt = now - lastTime
#                            lastTime = now
#                            if speed is None:
#                                speed = 1.0/dt
#                            else:
#                                speed = (t[k+1]-t[k])/dt
#                                averageSpeed = t[k+1]/(time()-initialTime)
                    def update():
                        nonlocal k
                        if k < len(t)-1:
                            partialInterval = numpy.linspace(0,t[k+1],500)
                            x_coord = geodesicParametrization(partialInterval).real
                            y_coord = geodesicParametrization(partialInterval).imag
                            plottedCurve.setData(x_coord,y_coord)
                            print("d = " + str(t[k+1]))
                            print("t = " + str(time()-initialTime))
                            #self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='k')   
#                            pos = numpy.empty((k+1,3))
#                            for i in range(0,k+1):
#                                pos[i]=(x_coord[i],y_coord[i],0)
#                            trial = gl.GLScatterPlotItem(pos=pos,color=(1,1,1,.3), size=1, pxMode=False)
#                            self.openGLWidget.addItem(trial)
                            k = (k+1)
                        if k == len(t)-1:
                            finalPointRed = pg.ScatterPlotItem([Q.real],[Q.imag],pen='r',brush = 'r')
                            self.PlotWidgetIn_pageUHP.addItem(finalPointRed)
                            print(time()-initialTime)
                            self.timer.stop()
#                        QtCore.QTimer.singleShot(1000*s/numberOfSteps, update)
 #                   update()
                    if self.timer:
                        self.timer.stop()
                        self.timer.deleteLater()
                    self.timer = QtCore.QTimer(self)
                    self.timer.timeout.connect(update)
                    self.timer.start(1000*s/numberOfSteps)
                    #print(numpy.ceil(s))
                
        #        timer = QtCore.QTimer(self)
        #        timer.timeout.connect(update)
        #        timer.start(250)
#                self.timer = QtCore.QTimer(self)
#                self.timer.timeout.connect(update)
#                self.timer.start(1)
                
#        if self.checkBoxUHPEnableClickOnCanvas.isChecked() == False:
#            twoClicks.clear()
            #print("as expected")
                
            #self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.disconnect(onClick) 
        
        
    
#        #p = self.PlotWidgetIn_pageCP
#        print(self.checkBoxUHPGMShowGeodesicSegments.isChecked())
#        def fuck(sl):
#                #print(self.PlotWidgetIn_pageUHP.plotItem.vb.pos())
#                #nonlocal p
#                print('something')
#                #print(sl.scenePos().x())
#                #print(sl.pos().x())
#                P=self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(sl.scenePos())
#                print(P.x())
#                print(P.y())
#                print(type(P.x()))
#                print(P.y())
#                #print (self.PlotWidgetIn_pageCP.plotItem.vb.mapSceneToView(sl))
#            
#        if self.checkBoxUHPGMShowGeodesicSegments.isChecked():
#            self.PlotWidgetIn_pageUHP.setEnabled(True)
#            
#
#
#    
#            self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(fuck)
#        else:
#            



        
#            
#            #self.PlotWidgetIn_pageUHP.scene().sigMouseMoved.connect(mouseMoved)
#            self.PlotWidgetIn_pageUHP.setEnabled(False)
#            #self.PlotWidgetIn_pageUHP.setEnabled(True)
#            print('yeah')
#            
#            



    def UHPGMGeodesicRayConstantRapidityAnimated(self,ev):# point-vector
        if self.radioButtonUHPGMGeoParamByArcLength.isChecked() == True and self.radioButtonUHPGMPointVector.isChecked() == True and self.stackedWidgetIn_pageUHP.currentIndex() == 1:
            global arbManyClicks
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y <= 0:
                pass
            else:
                arbManyClicks.append(x + y*(1j))
                Point = pg.ScatterPlotItem([x],[y],pen='r',brush = 'r')
                self.PlotWidgetIn_pageUHP.addItem(Point)
                def UHPGMGetVector(evt):
                    if len(arbManyClicks) == 0:
                        pass
                    else:
                        startPoint = arbManyClicks[-1]
                        u = self.PlotWidgetIn_pageUHPGeodesicMotion.plotItem.vb.mapSceneToView(evt.scenePos()).x()
                        v = self.PlotWidgetIn_pageUHPGeodesicMotion.plotItem.vb.mapSceneToView(evt.scenePos()).y()
    #                    Line = pg.InfiniteLine(pos=u, angle=90, pen=pg.mkPen('k', width=self.widthBoundingLineUHP))
    #                    self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(Line)
                        initialVector = u + v*(1j)
                        parametrization = UHP_HP.UHPGeodesicMotion().UHPGeodesicSegmentConstantRapidity(startPoint,initialVector)
                        theDrawing = self.PlotWidgetIn_pageUHP.plot(pen = self.blackPenWidth2)
                        theCircle = self.PlotWidgetIn_pageUHP.plot(pen = self.blackPenWidth2)
                        numberOfSteps = 100
                        k = 0
                        def update():
                            nonlocal k
                            if k < numberOfSteps:
                                partialInterval = numpy.linspace(0,k+1,(k+1)*50)
                                x_coord = parametrization(partialInterval).real
                                y_coord = parametrization(partialInterval).imag
                                theDrawing.setData(x_coord,y_coord)
                                #PointBlue = pg.ScatterPlotItem([x_coord[-1]],[y_coord[-1]],pen='m',brush = 'm')
                                #self.PlotWidgetIn_pageUHP.addItem(PointBlue)
                                if self.checkBoxUHPGMwave.isChecked() == True:
                                    try:
                                        hRadius = UHP_HP.UHPBasics().UHPDist(startPoint,x_coord[-1]+y_coord[-1]*(1j))
                                        eCenterAndRadius = UHP_HP.UHPBasics().eCenterAndRadiusH_Circle(startPoint,hRadius)
                                        eCenter = eCenterAndRadius[0]
                                        eRadius = eCenterAndRadius[1]
                                        theCircle.setData(eCenter.real+eRadius*numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),eCenter.imag+eRadius*numpy.sin(numpy.linspace(0,2*numpy.pi,1000)))
                                    except:
                                        pass
                                k = k+1
                            if k == numberOfSteps:
                                x_coord = parametrization(k+1).real
                                y_coord = parametrization(k+1).imag
                                finalPointRed = pg.ScatterPlotItem([x_coord],[y_coord],pen='r',brush = 'r')
                                self.PlotWidgetIn_pageUHP.addItem(finalPointRed)
                                self.timer.stop()
#                            QtCore.QTimer.singleShot(10, update)
#                        update()
                        if self.timer:
                            self.timer.stop()
                            self.timer.deleteLater()
                        self.timer = QtCore.QTimer(self)
                        self.timer.timeout.connect(update)
                        self.timer.start(10)

                if len(arbManyClicks) == 1:
                    self.PlotWidgetIn_pageUHPGeodesicMotion.scene().sigMouseClicked.connect(UHPGMGetVector)
#                else:
#                    pass

                

                            
                
            

    def UHPCMCircMotionAntiClockwise(self,ev):
        if self.stackedWidgetIn_pageUHP.currentIndex() == 2:
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y <= 0:
                pass
            else:
                centerDot = pg.ScatterPlotItem([x],[y],pen='r',brush = 'r')
                self.PlotWidgetIn_pageUHP.addItem(centerDot)
                def effectOf_pushButtonUHPCMCircMotionAntiClockwise():
                    self.PlotWidgetIn_pageUHP.addItem(centerDot)
                    hCenter = x+y*(1j)
                    hRadius = float(self.horizontalSliderUHPCMhRadius.value())
                    angularSpeed = int(self.horizontalSliderUHPCMangularSpeed.value())*2*numpy.pi/360
                    eCenterAndRadius = UHP_HP.UHPBasics().eCenterAndRadiusH_Circle(hCenter,hRadius)
                    eCenter = eCenterAndRadius[0]
                    eRadius = eCenterAndRadius[1]
                    curve = UHP_HP.UHPCircularMotion().UHPCircSegmentParamByArcLength(eCenter,eRadius,-numpy.pi,numpy.pi)[0]
                    length = UHP_HP.UHPCircularMotion().UHPCircSegmentParamByArcLength(eCenter,eRadius,-numpy.pi,-numpy.pi+angularSpeed)[1]
                    theSegment = self.PlotWidgetIn_pageUHP.plot(pen=self.blackPenWidth2)
                    movingDot = pg.ScatterPlotItem([curve(0).real],[curve(0).imag])
                    self.PlotWidgetIn_pageUHP.addItem(movingDot)
                    numberOfSteps = 50
                    t = numpy.linspace(0,5*length,numberOfSteps+1)
                    k = 0
                    def update():
                        nonlocal hCenter
                        nonlocal k
                        if k < len(t)-1:
                            movingDot.setData([curve(t[k+1]).real],[curve(t[k+1]).imag])
                            partialInterval = numpy.linspace(0,t[k+1],k*100)
                            x_coord,y_coord = curve(partialInterval).real, curve(partialInterval).imag
                            theSegment.setData(x_coord,y_coord)
                            currentPt= curve(t[k])
                            radialGeodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(hCenter,currentPt)
                            self.PlotWidgetIn_pageUHP.plot(radialGeodesicSegment.real,radialGeodesicSegment.imag,pen='r')
                            k = k+1
                        if k == len(t)-1:
                            self.timer.stop()
                    if self.timer:
                        self.timer.stop()
                        self.timer.deleteLater()
                    self.timer = QtCore.QTimer(self)
                    self.timer.timeout.connect(update)
                    self.timer.start(10)


                self.pushButtonUHPCMCircMotionAntiClockwise.clicked.connect(effectOf_pushButtonUHPCMCircMotionAntiClockwise)
            
        
            
#    def effectOf_pushButtonUHPCMCircMotionAntiClockwise(self):
#        curve = UHP_HP.UHPCircularMotion().UHPCircSegmentParamByArcLength(200j,150,-numpy.pi,numpy.pi)[0]
#        length = UHP_HP.UHPCircularMotion().UHPCircSegmentParamByArcLength(200j,150,-numpy.pi,numpy.pi)[1]
#        numberOfSteps = 500
#        t = numpy.linspace(0,5*length,numberOfSteps+1)
#        hcenter = numpy.sqrt(200**2-150**2)*(1j)
#        k=0
#        def update():
#            nonlocal k
#            if k < len(t)-1:
#                partialInterval = numpy.linspace(t[k],t[k+1],100)
#                x_coord = curve(partialInterval).real
#                y_coord = curve(partialInterval).imag
#                self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='k')
#                currentPt= curve(t[k])
#                radialGeodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(hcenter,currentPt)
#                self.PlotWidgetIn_pageUHP.plot(radialGeodesicSegment.real,radialGeodesicSegment.imag,pen='r')
#                nextPt= curve(t[k+1])
#                radialGeodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(hcenter,nextPt)
#                self.PlotWidgetIn_pageUHP.plot(radialGeodesicSegment.real,radialGeodesicSegment.imag,pen='k')
#                k = (k+1)
#            if k == len(t):
#                self.timer.stop()
##            QtCore.QTimer.singleShot(1000*numpy.ceil(numpy.abs(length))/numberOfSteps, update)
##        update()
#        if self.timer:
#            self.timer.stop()
#            self.timer.deleteLater()
#        self.timer = QtCore.QTimer(self)
#        self.timer.timeout.connect(update)
#        self.timer.start(1000*numpy.ceil(numpy.abs(length))/numberOfSteps)
#
#        
#            
            
#####
#####


    def UHPIsomsSpecificIdealPolygon(self):
        if self.stackedWidgetIn_pageUHP.currentIndex() == 3:
            g = int(self.spinBoxUHPIsomsGenus.cleanText())
            p = int(self.spinBoxUHPIsomsNumOfPuncts.cleanText())
            curvesAndColors = UHP_HP.UHPFuchsianRepresentative().UHPSidesOfSpecificIdealPolygon(g,p)
            NumOfSides = (4*g) + (2*(p-1))
            for k in range(NumOfSides):
                x_coord = (curvesAndColors["curves"])[k].real
                y_coord = (curvesAndColors["curves"])[k].imag
                color = (curvesAndColors["curvesColors"])[k]
                theDrawing = pg.PlotCurveItem(x_coord,y_coord,pen=pg.mkPen(str(color), width=2))
                self.PlotWidgetIn_pageUHP.addItem(theDrawing)
        



























#################################
#################################
#################################
### HYPERBOLIC PLANE
### UPPER HALF PLANE --- PD

    def effectOf_pushButtonPDClearCanvas(self):
        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
            self.timer = None        
        self.PlotWidgetIn_pagePD.clear()
        self.PDdraggableDotsStaticGeodSegs.setData(pos=numpy.array([[0,-100]]))
        self.PDdraggableDotsConvexHull.setData(pos=numpy.array([[0,-100]]))
        self.PDdraggableDotsMidPtForSidePairing.setData(pos=numpy.array([[0,-100]]))
        self.PlotWidgetIn_pagePD.addItem(self.PDdraggableDotsStaticGeodSegs)
        self.PlotWidgetIn_pagePD.addItem(self.PDdraggableDotsConvexHull)
        self.PlotWidgetIn_pagePD.addItem(self.PDdraggableDotsMidPtForSidePairing)
        self.PlotWidgetIn_pagePD.addItem(self.boundingCirclePD)
        self.boundingCirclePD.setPen(pg.mkPen('k',width=2))
        for clicked in Clicks:
            clicked.clear()





        

    def PDmouseMoved(self,evt):
        pos = evt[0]  ## using signal proxy turns original arguments into a tuple
#        if self.PlotWidgetIn_pageUHP.sceneBoundingRect().contains(pos):
        mousePoint = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(pos)
        #index = int(mousePoint.x())
        if mousePoint.x()**2+mousePoint.y()**2 < 1:
            self.boundingCirclePD.setPen(pg.mkPen('k',width=2))
            self.labelPDxNumber.setText("<span style='font-size: 12pt'><span style='color: green'>x=%0.100f" % (mousePoint.x()))
            self.labelPDyNumber.setText("<span style='font-size: 12pt'><span style='color: green'>y=%0.100f" % (mousePoint.y()))
            self.labelPDNormNumber.setText("<span style='font-size: 12pt'><span style='color: green'>Norm=%0.100f" % (mousePoint.x()**2+mousePoint.y()**2))

        #index = int(mousePoint.x())
        if mousePoint.x()**2+mousePoint.y()**2 == 1:
            self.boundingCirclePD.setPen(pg.mkPen('m',width=2))
            self.labelPDxNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>x=%0.100f" % (mousePoint.x()))
            self.labelPDyNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>y=%0.100f" % (mousePoint.y()))
            self.labelPDNormNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>Norm=%0.100f" % (mousePoint.x()**2+mousePoint.y()**2))
        if mousePoint.x()**2+mousePoint.y()**2 > 1:
            self.boundingCirclePD.setPen(pg.mkPen('r',width=2))
            self.labelPDxNumber.setText("<span style='font-size: 12pt'><span style='color: red'>--")
            self.labelPDyNumber.setText("<span style='font-size: 12pt'><span style='color: red'>--")
            self.labelPDNormNumber.setText("<span style='font-size: 12pt'><span style='color: red'>--")

#        print(mousePoint.x())
#        print(mousePoint.y())
#        print("yes")

    def PDGMmouseMoved(self,evt):
        pos = evt[0]
        mousePoint = self.PlotWidgetIn_pagePDGeodesicMotion.plotItem.vb.mapSceneToView(pos)
        self.horLineCrosshairPDGM.setPos(mousePoint.y())
        self.vertLineCrosshairPDGM.setPos(mousePoint.x())
































#    def PDBCGeodesicSegmentStatic(self,ev):
#        if self.checkBoxPDEnableClickOnCanvas.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 0:
#            global twoClicks
#            x = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).x()
#            y = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).y()
#            if x**2 + y**2 > 1:
#                pass
#            else:
#                while len(twoClicks) < 3:  
#                    twoClicks.append([x,y])
#                #print(twoClicks)
#                if len(twoClicks) == 3:  
#                    del twoClicks[0]
#                #print(twoClicks)
#                if twoClicks[0] == twoClicks[1]:
#                    initialPoint = pg.ScatterPlotItem([twoClicks[0][0]],[twoClicks[0][1]])
#                    self.PlotWidgetIn_pagePD.addItem(initialPoint)
#                    self.lineEditPDBCComplexNumber1.setText(str(twoClicks[0][0]+twoClicks[0][1]*(1j)))
#                else:
#                    #print(twoClicks)
#                    finalPoint = pg.ScatterPlotItem([twoClicks[1][0]],[twoClicks[1][1]])
#                    self.PlotWidgetIn_pagePD.addItem(finalPoint)
#                    P = twoClicks[0][0]+twoClicks[0][1]*(1j)
#                    Q = twoClicks[1][0]+twoClicks[1][1]*(1j)
#                    self.lineEditPDBCComplexNumber2.setText(str(Q))
#                    #print(P,Q)
#                    geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q)
#                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
#                    self.PlotWidgetIn_pagePD.plot(x_coord,y_coord,pen='k')
#                    self.labelPDBChdistancenumber.setNum(PD_HP.PDBasics().PDDist(P,Q))
#        if self.checkBoxPDEnableClickOnCanvas.isChecked() == False:
#            twoClicks.clear()
#            print("as expected")        

    def PDBCGeodesicSegmentStatic(self,ev):
        if self.radioButtonPDBCGeodesicSegments.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 0:
            global arbManyClicks
            global auxStorage
            x = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if x**2+y**2 > 1:
                pass
            else:
                arbManyClicks.append([x,y])
                if len(arbManyClicks)==1:
                    points = numpy.array([[arbManyClicks[0][0],arbManyClicks[0][1]]],dtype=float)
                    self.PDdraggableDotsStaticGeodSegs.setData(pos=points,  pxMode=True)
                else:
                    points = numpy.array([[arbManyClicks[k][0],arbManyClicks[k][1]] for k in range(len(arbManyClicks))],dtype=float)
                    self.PDdraggableDotsStaticGeodSegs.setData(pos=points,  pxMode=True)
                    P = arbManyClicks[-2][0]+arbManyClicks[-2][1]*(1j)
                    Q = arbManyClicks[-1][0]+arbManyClicks[-1][1]*(1j)
                    geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q)
                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                    drawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                    self.PlotWidgetIn_pagePD.addItem(drawing)
                    auxStorage.append(drawing)
#                    self.labelPDBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q))
                    
#    @QtCore.pyqtSlot(object,int)
    def PDBCDragGeodesicSegment(self,pt,ind): # PERSONAL NOTE: there is a bad math/programming practice in draggableDots.PDdraggableDot().mouseDragEvent
        global arbManyClicks
        global auxStorage
        if len(arbManyClicks) < 2 or ind == -1 or pt[0]**2+pt[1]**2>1:
            pass
        else:
            P = pt[0]+pt[1]*(1j)
            #print(pt[0]**2+pt[1]**2)
            if ind == 0:
                neighbour = arbManyClicks[1]
                Q = neighbour[0]+neighbour[1]*(1j)
                curve = auxStorage[0]   
                geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
                self.labelPDBChdistancenumber.setNum(PD_HP.PDBasics().PDDist(P,Q))
            if ind == len(arbManyClicks)-1:
                neighbour = arbManyClicks[ind-1]
                Q = neighbour[0]+neighbour[1]*(1j)
                curve = auxStorage[ind-1]
                geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(Q,P)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
                self.labelPDBChdistancenumber.setNum(PD_HP.PDBasics().PDDist(Q,P))
            if 0 < ind and ind < len(arbManyClicks)-1:
                neighbour1 = arbManyClicks[ind-1]
                neighbour2 = arbManyClicks[ind+1]
                Q1 =neighbour1[0]+neighbour1[1]*(1j)
                Q2 = neighbour2[0]+neighbour2[1]*(1j)
                curve1 = auxStorage[ind-1]
                curve2 = auxStorage[ind]
                geodesicSegment1 = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q1)
                geodesicSegment2 = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q2)
                x_coord1, y_coord1 = geodesicSegment1.real, geodesicSegment1.imag
                x_coord2, y_coord2 = geodesicSegment2.real, geodesicSegment2.imag
                curve1.setData(x_coord1,y_coord1)
                curve2.setData(x_coord2,y_coord2)
                #self.labelUHPBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q1))
            arbManyClicks[ind] = [pt[0],pt[1]]
#            arbManyClicks.remove(arbManyClicks[ind])
#            arbManyClicks.insert(ind,[pt[0],pt[1]])
                




    def PDBCConvexHull(self,ev):
        if self.radioButtonPDBCConvexHull.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 0:
            global arbManyClicks
            global auxStorage
            x = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if x**2+y**2>1:
                pass
            else:
                arbManyClicks.append(x+y*(1j))
                if len(arbManyClicks) == 1:
                    points = numpy.array([[x,y]],dtype=float)
                    self.PDdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
                    auxStorage.append(x+y*(1j))
                elif len(arbManyClicks) == 2:
                    points = numpy.array([[arbManyClicks[k].real,arbManyClicks[k].imag] for k in range(len(arbManyClicks))],dtype=float)
                    self.PDdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
                    auxStorage.append(x+y*(1j))
                    geodesicSeg = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(arbManyClicks[0],arbManyClicks[1])
                    x_coord, y_coord = geodesicSeg.real, geodesicSeg.imag
                    drawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                    self.PlotWidgetIn_pagePD.addItem(drawing)
                    auxStorage2.append(drawing)
                else:
                    if PD_HP.PDBasics().is_point_in_h_convex_hull(x+y*(1j),auxStorage) == False:
                        auxStorage.append(x+y*(1j))
                    vertices = PD_HP.PDBasics().verts_h_polygon_counter_clockwise(auxStorage)
                    points = numpy.array([[arbManyClicks[k].real,arbManyClicks[k].imag] for k in range(len(arbManyClicks))],dtype=float)
                    self.PDdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
                    for theCurves in auxStorage2:
                        self.PlotWidgetIn_pagePD.removeItem(theCurves)
                    auxStorage2.clear()
                    for i in range(0,len(vertices)-1,1):
                        geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(vertices[i],vertices[i+1])
                        x_coord = geodesicSegment.real
                        y_coord = geodesicSegment.imag
                        theDrawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                        self.PlotWidgetIn_pagePD.addItem(theDrawing)
                        auxStorage2.append(theDrawing)
#                    brushes = [0.5, (100, 100, 255), 0.5]
#                    fills = [pg.FillBetweenItem(auxStorage2[i], auxStorage2[i+1], brushes[1]) for i in range(0,len(vertices)-2,1)]
#                    for f in fills:
#                        self.PlotWidgetIn_pagePD.addItem(f)
                    auxStorage.clear()
                    auxStorage.extend(vertices)
                
    def PDBCDragConvexHull(self,pt,ind):
        global arbManyClicks
        global auxStorage
        global auxStorage2
        if len(arbManyClicks) < 2 or ind == -1 or pt[0]**2+pt[1]**2 > 1:
            pass
        else:
            try:
                P = pt[0]+pt[1]*(1j)
                print(numpy.absolute(P))
                if len(arbManyClicks) == 2:
                    Q = arbManyClicks[(ind+1)%2]
                    curve = auxStorage2[0]   
                    geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q)
                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                    curve.setData(x_coord,y_coord)
                    auxStorage.remove(auxStorage[ind])
                    auxStorage.insert(ind,P)
                    arbManyClicks.remove(arbManyClicks[ind])
                    arbManyClicks.insert(ind,P)
                if len(arbManyClicks) > 2:
                    arbManyClicks.remove(arbManyClicks[ind])
                    arbManyClicks.insert(ind,P)
                    vertices = PD_HP.PDBasics().verts_h_polygon_counter_clockwise(arbManyClicks)
                    for Drawing in auxStorage2:
                        self.PlotWidgetIn_pagePD.removeItem(Drawing)
                    auxStorage2.clear()                    
                    for i in range(0,len(vertices)-1,1):
                        geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(vertices[i],vertices[i+1])
                        x_coord = geodesicSegment.real
                        y_coord = geodesicSegment.imag
                        theDrawing = pg.PlotCurveItem(x_coord,y_coord,pen=self.blackPenWidth2)
                        self.PlotWidgetIn_pagePD.addItem(theDrawing)
                        auxStorage2.append(theDrawing)                
                    auxStorage.clear()
                    auxStorage.extend(vertices)
            except:
                pass













    
    def PDGMGeodesicSegmentAnimated(self,ev): # point-point
        if self.radioButtonPDGMGeoParamByArcLength.isChecked() == True and self.radioButtonPDGMPointPoint.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 1:
            global twoClicks
            x = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if x**2 + y**2 > 1:
                pass
            else:
                while len(twoClicks) < 3:  
                    twoClicks.append([x,y])
                if len(twoClicks) == 3:  
                    del twoClicks[0]
                #print(twoClicks)
                if twoClicks[0] == twoClicks[1]:
                    initialPoint = pg.ScatterPlotItem([twoClicks[0][0]],[twoClicks[0][1]],pen='r',brush='r')
                    self.PlotWidgetIn_pagePD.addItem(initialPoint)
                    self.lineEditPDGMComplexNumber1.setText(str(twoClicks[0][0]+twoClicks[0][1]*(1j)))
                else:
                    #print(twoClicks)
                    finalPoint = pg.ScatterPlotItem([twoClicks[1][0]],[twoClicks[1][1]])
                    self.PlotWidgetIn_pagePD.addItem(finalPoint)
                    P = twoClicks[0][0]+twoClicks[0][1]*(1j)
                    Q = twoClicks[1][0]+twoClicks[1][1]*(1j)
                    self.lineEditPDGMComplexNumber2.setText(str(Q))
                    #print(P,Q)
                    geodesicParametrization = PD_HP.PDGeodesicMotion().PDGeodesicSegmentParamByArcLength(P,Q)[0]
                    s = PD_HP.PDGeodesicMotion().PDGeodesicSegmentParamByArcLength(P,Q)[1]
                    self.labelPDGMhdistancenumber.setNum(s)
                    numberOfSteps = 500
                    t = numpy.linspace(0,s,numberOfSteps+1)
                    plottedCurve = self.PlotWidgetIn_pagePD.plot(pen = self.blackPenWidth2)
                    k=0
                    def update():
                        nonlocal k
                        if k < len(t)-1:
                            z_k = geodesicParametrization(t[k+1])
                            partialSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,z_k)
                            x_coord = partialSegment.real
                            y_coord = partialSegment.imag
                            plottedCurve.setData(x_coord,y_coord)
                        k = (k+1)
                        if k == len(t)-2:
                            finalPointRed = pg.ScatterPlotItem([Q.real],[Q.imag],pen='r',brush = 'r')
                            self.PlotWidgetIn_pagePD.addItem(finalPointRed)
                            self.timer.stop()
                            
                    
                    if self.timer:
                        self.timer.stop()
                        self.timer.deleteLater()
                    self.timer = QtCore.QTimer(self)
                    self.timer.timeout.connect(update)
                    self.timer.start(1000*s/numberOfSteps)



        
        
        
        
        
        
        
    def PDGMGeodesicRayConstantRapidityAnimated(self,ev):# point-vector
        if self.radioButtonPDGMGeoParamByArcLength.isChecked() == True and self.radioButtonPDGMPointVector.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 1:
            global arbManyClicks
            x = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if x**2+y**2 >= 1:
                pass
            else:
                try:
                    arbManyClicks.append(x + y*(1j))
                    Point = pg.ScatterPlotItem([x],[y],pen='r',brush = 'r')
                    self.PlotWidgetIn_pagePD.addItem(Point)
                    def PDGMGetVector(evt):
                        if len(arbManyClicks) == 0:
                            pass
                        else:
                            startPoint = arbManyClicks[-1]
                            u = self.PlotWidgetIn_pagePDGeodesicMotion.plotItem.vb.mapSceneToView(evt.scenePos()).x()
                            v = self.PlotWidgetIn_pagePDGeodesicMotion.plotItem.vb.mapSceneToView(evt.scenePos()).y()
                            initialVector = u + v*(1j)
                            parametrization = PD_HP.PDGeodesicMotion().PDGeodesicSegmentConstantRapidity(startPoint,initialVector)
                            theDrawing = self.PlotWidgetIn_pagePD.plot(pen = self.blackPenWidth2)
                            theCircle = self.PlotWidgetIn_pagePD.plot(pen = self.blackPenWidth2)
                            numberOfSteps = 100
                            rapidity = PD_HP.PDBasics().PDNorm(startPoint,initialVector)
                            s = rapidity
                            t = numpy.linspace(0,s,numberOfSteps+1)
                            k = 0
                            def update():
                                nonlocal k
                                if k < len(t)-1:
                                    z_k = parametrization(t[k+1])
                                    partialSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(startPoint,z_k)
                                    x_coord = partialSegment.real
                                    y_coord = partialSegment.imag
                                    theDrawing.setData(x_coord,y_coord)
                                    if self.checkBoxPDGMwave.isChecked() == True:
                                        try:
                                            hRadius = PD_HP.PDBasics().PDDist(startPoint,z_k)
                                            eCenterAndRadius = PD_HP.PDBasics().eCenterAndRadiusH_Circle(startPoint,hRadius)
                                            eCenter = eCenterAndRadius[0]
                                            eRadius = eCenterAndRadius[1]
                                            theCircle.setData(eCenter.real+eRadius*numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),eCenter.imag+eRadius*numpy.sin(numpy.linspace(0,2*numpy.pi,1000)))
                                        except:
                                            pass
                                    k = k+1
                                if k == len(t)-2:
                                    x_coord = parametrization(t[k]).real
                                    y_coord = parametrization(t[k]).imag
                                    finalPointRed = pg.ScatterPlotItem([x_coord],[y_coord],pen='r',brush = 'r')
                                    self.PlotWidgetIn_pagePD.addItem(finalPointRed)
                                    self.timer.stop()
    
                            if self.timer:
                                self.timer.stop()
                                self.timer.deleteLater()
                            self.timer = QtCore.QTimer(self)
                            self.timer.timeout.connect(update)
                            self.timer.start(50)
    
                    if len(arbManyClicks) == 1:
                        self.PlotWidgetIn_pagePDGeodesicMotion.scene().sigMouseClicked.connect(PDGMGetVector) 
                        
                except:
                    pass
                        
                    
                    
                    
                    
                    
                    
                    
    def PDIsomsSpecificIdealPolygon(self):
        if self.stackedWidgetIn_pagePD.currentIndex() == 3:
            g = int(self.spinBoxPDIsomsGenus.cleanText())
            p = int(self.spinBoxPDIsomsNumOfPuncts.cleanText())
            ordersText = self.lineEditPDIsomsOrdersOfOrbPts.text()
            orders = []
            commaPositions = []
            for t in range(len(ordersText)):
                if str(ordersText[t]) == ",":
                    commaPositions.append(t)
            if len(commaPositions) == 0 and len(ordersText)!=0:
                string = ""
                for l in range(len(ordersText)):
                    string = string + str(ordersText[l])
                orders.append(int(string))
            for s in range(len(commaPositions)):
                if s == 0 and commaPositions[s] != 0:
                    string = ""                    
                    for l in range(commaPositions[s]):
                        string = string + str(ordersText[l])
                    orders.append(int(string))
                if s > 0 and commaPositions[s] != commaPositions[s-1]+1:
                    string = ""
                    for l in range(commaPositions[s-1]+1,commaPositions[s]):
                        string = string + str(ordersText[l])
                    orders.append(int(string))
                if s == len(commaPositions)-1 and commaPositions[s] != len(ordersText)-1:
                    string = ""
                    for l in range(commaPositions[s]+1,len(ordersText)):
                        string = string + str(ordersText[l])
                    orders.append(int(string))         
            orders = [j for j in orders if j > 1]        
            #print(orders)
            triplesOfPtsAndColors = PD_HP.PDFuchsianRepresentative().PDSidesOfSpecificIdealPolygon(g,p,orders)
            NumOfSides = (4*g) + (2*(p-1)) + (2*len(orders))#2g-1+4g+2(p-1)=6g+2p-3
            #print(NumOfSides)
            drawnCurvesList = []
            sidePairings = PD_HP.PDFuchsianRepresentative().PDSidePairingsOfSpecificIdealPolygon(g,p,orders)
            #self.PDdraggableDotsMidPtForSidePairing.setData(pos=points, brush = 'k',  pxMode=True)

            curvesToSidePairingDict = {}
            firstFace = []

            for k in range(NumOfSides):
                kthTriple = (triplesOfPtsAndColors["triplesOfPtsOnCurves"][k])[0]
                kthGeodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(kthTriple[0],kthTriple[2])
                x_coord = kthGeodesicSegment.real
                y_coord = kthGeodesicSegment.imag
                colour = (triplesOfPtsAndColors["curvesColors"])[k]
                theDrawing = pg.PlotCurveItem(x_coord,y_coord,pen=pg.mkPen(color=colour, width=2),clickable=True)
                self.PlotWidgetIn_pagePD.addItem(theDrawing)
                drawnCurvesList.append(theDrawing)
                
                
                # THE NEXT FEW LINES ADD GEODESIC SEGMENTS THAT INDICATE THE SIDE PAIRINGS
                kthMidPt = kthTriple[1]
                a, b, c, d = (sidePairings[k][0]**(-1))[0,0], (sidePairings[k][0]**(-1))[0,1], (sidePairings[k][0]**(-1))[1,0], (sidePairings[k][0]**(-1))[1,1]
                imageOfkthMidPt = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(a,b,c,d)(kthMidPt)
                geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(kthMidPt,imageOfkthMidPt)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                drawing = pg.PlotCurveItem(x_coord,y_coord,pen=pg.mkPen(color=colour, width=1))
                self.PlotWidgetIn_pagePD.addItem(drawing)
                
                matrix = numpy.matrix([[a,b],[c,d]])**(-1)
                alpha, beta, gamma, delta = matrix[0,0], matrix[0,1], matrix[1,0], matrix[1,1]
                curvesToSidePairingDict[theDrawing]=[matrix,colour,[kthTriple[0],kthTriple[2]],Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(alpha,beta,gamma,delta)(0)]
                firstFace.append(theDrawing)

                
            Faces = [firstFace]
                

                
#            points = numpy.array([[(curvesAndColors["midpoints"])[k][0].real,(curvesAndColors["midpoints"])[k][0].imag] for k in range(NumOfSides)],dtype=float)
#            self.PDdraggableDotsConvexHull.setData(pos=points, brush = 'k',  pxMode=True)
            
#            brushes = ["r", (100, 100, 255), "b"]
#            fills = [pg.FillBetweenItem(drawnCurvesList[k][0], drawnCurvesList[k+1][0], brushes[k%3]) for k in range(NumOfSides-1)]
#            for f in fills:
#                self.PlotWidgetIn_pagePD.addItem(f)
            # randomChoiceOfColors = numpy.random.choice(5, 3, replace=False)
            x, y = numpy.array([[0,0],[0,0]])
            fills = [pg.FillBetweenItem(pg.PlotCurveItem(x,y,pen=pg.mkPen(color="c", width=1)), theCurve, "c") for theCurve in drawnCurvesList]
            for f in fills:
                self.PlotWidgetIn_pagePD.addItem(f)

            
            
            def plotClicked(curve):####
                #nonlocal drawnCurvesList
                for c in drawnCurvesList:
                    if c is curve:
                        c.setPen(pg.mkPen(color=(curvesToSidePairingDict[c])[1], width=4))
                        cFaces = []
                        for face in Faces:
                            if c in face:
                                cFaces.append(face)
                        if len(cFaces) == 1:
                            matrix = (curvesToSidePairingDict[c])[0]
                            Mobius = Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(matrix[0,0],matrix[0,1],matrix[1,0],matrix[1,1])
                            newcFace = []
                            for side in cFaces[0]:
                                point1 = Mobius(curvesToSidePairingDict[side][2][0])
                                point2 = Mobius(curvesToSidePairingDict[side][2][1])
                                newCurve = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(point1,point2)
                                xCoord, yCoord = newCurve.real, newCurve.imag
                                newDrawing = pg.PlotCurveItem(xCoord,yCoord,pen=pg.mkPen(color=(curvesToSidePairingDict[side])[1], width=2),clickable=True)
                                self.PlotWidgetIn_pagePD.addItem(newDrawing)
                                drawnCurvesList.append(newDrawing)
                                newcFace.append(newDrawing)
                                B = (curvesToSidePairingDict[side])[0]
                                newMatrix = matrix*B*(matrix**(-1))
                                newAlpha,newBeta,newGamma,newDelta = newMatrix[0,0], newMatrix[0,1], newMatrix[1,0], newMatrix[1,1]
                                curvesToSidePairingDict[newDrawing]=[newMatrix,(curvesToSidePairingDict[side])[1],[point1,point2],Mobius_CP.MobiusAssocToMatrix().EvaluationAtConcretePoint(newAlpha,newBeta,newGamma,newDelta)((curvesToSidePairingDict[c])[3])]
                                newDrawing.sigClicked.connect(plotClicked)
                                
                                #x, y = numpy.array([[curvesToSidePairingDict[c][3].real,curvesToSidePairingDict[c][3].imag],[curvesToSidePairingDict[c][3].real,curvesToSidePairingDict[c][3].imag]])
                                #fills = [pg.FillBetweenItem(pg.PlotCurveItem(x,y,pen=pg.mkPen(color="r", width=1)), c, "r")]
                                #for f in fills:
                                #    self.PlotWidgetIn_pagePD.addItem(f)

                                
                            Faces.append(newcFace)
                    else:
                        c.setPen(pg.mkPen(color=(curvesToSidePairingDict[c])[1], width=2))
                        
            for curve in drawnCurvesList:
                curve.sigClicked.connect(plotClicked)
    
                    
                    
                    
                    
                    
                    
                    
                    
                    
        
app = QtWidgets.QApplication(sys.argv)
form = appMainWindow()
form.show()
app.exec_()

#if __name__ == "__main__":            
#    app = QtWidgets.QApplication(sys.argv)
#    form = appMainWindow()
#    form.show()
#    app.exec_()