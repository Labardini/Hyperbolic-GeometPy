#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan  6 15:53:06 2018
@author: daniellabardini
"""



from PyQt5 import QtWidgets
#from PyQt5.QtCore import *
#from PyQt5.QtGui import *
import sys

import numpy


import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

from pyqtgraph.ptime import time

#### FOR HYPERBOLOID
import pyqtgraph.opengl as gl
####





import appWindowDesign
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
Clicks = [oneClick,twoClicks,threeClicks,arbManyClicks,auxStorage]




        

       









class appMainWindow(QtWidgets.QDialog, appWindowDesign.Ui_MainWindow):
    
    def __init__(self, parent=None):
        super(appMainWindow,self).__init__(parent)
        self.setupUi(self)
        
        self.timer = None # in some animations it will become QtCore.QTimer(self)

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
        self.boundingLineUHP = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('g', width=self.widthBoundingLineUHP))
        self.PlotWidgetIn_pageUHP.addItem(self.boundingLineUHP)
        self.UHPdraggableDots = dD.draggableDot()
        self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDots)
        

        
  
        self.PlotWidgetIn_pageUHPGeodesicMotion.setXRange(-2,2)
        self.PlotWidgetIn_pageUHPGeodesicMotion.setYRange(-2,2)
        self.PlotWidgetIn_pageUHPGeodesicMotion.hideAxis('left')
        self.PlotWidgetIn_pageUHPGeodesicMotion.showAxis('right')
        self.PlotWidgetIn_pageUHP.disableAutoRange()
        self.PlotWidgetIn_pageUHP.setAspectLocked(1.0)
        self.widthLinesUHPGM = 2
        self.boundingCircleUHPGM = pg.PlotCurveItem(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen=pg.mkPen('b',width=self.widthLinesUHPGM), clickable=True)
        self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(self.boundingCircleUHPGM)
        self.horLineCrosshairUHPGM = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('g', width=self.widthLinesUHPGM))
        self.vertLineCrosshairUHPGM = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('g', width=self.widthLinesUHPGM))
        self.horLineUHPGM = pg.InfiniteLine(angle=0, movable = False, pen=pg.mkPen('b',width=self.widthLinesUHPGM))
        self.vertLineUHPGM = pg.InfiniteLine(angle=90, movable = False, pen=pg.mkPen('b',width=self.widthLinesUHPGM))
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
        self.boundingCirclePD = pg.PlotCurveItem(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen='g', clickable=True)
        self.PlotWidgetIn_pagePD.addItem(self.boundingCirclePD)
        
        
#        self.PlotWidgetIn_pagePD.plot(numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),numpy.sin(numpy.linspace(0,2*numpy.pi,1000)),pen='w')
#        self.circROIPD = pg.CircleROI([-1, -1], [2, 2], pen=(4,100), maxBounds=[2,2])
#        self.PlotWidgetIn_pagePD.addItem(self.circROIPD)
#        self.circROIPD.removeHandle(0)








####################
####################
####### DISPLAY HYPERBOLOID


        self.openGLWidget.opts['viewport'] =  (0, 0, 1100, 900)
        #self.openGLWidget.showMaximized()
        self.openGLWidget.setCameraPosition(distance=50)
        
        

        ## Add a grid to the view
        g = gl.GLGridItem()
        #g.scale(2,2,1)
        #g.setDepthValue(10)  # draw grid after surfaces since they may be translucent
        g.setSize(50,50,50)
        g.translate(20,0,-30)
        self.openGLWidget.addItem(g)
        
                
        x = numpy.linspace(-20, 20, 100)
        y = numpy.linspace(-20, 20, 100)
        z = numpy.sqrt((x.reshape(100,1) ** 2) + (y.reshape(1,100) ** 2) + 1)
#        x = numpy.linspace(-50, 50, 100)
#        y = numpy.linspace(-50, 50, 100)
#        z = numpy.sqrt((x.reshape(100,1) ** 2) + (y.reshape(1,100) ** 2) + 1)
        p2 = gl.GLSurfacePlotItem(x=x, y=y, z=z, shader='shaded', color=(0.5, 0.5, 1, 1))
        p2.translate(20,0,-30)
        self.openGLWidget.addItem(p2)










#####
##### Daniel's own configuration of some buttons, signals and slots,
##### for navigation through the app's pages
##### DEFINITIONS OF CONNECTIONS FOR SIGNALS
##### SEE BELOW FOR THE DEFINITIONS OF THE SIGNALS
        
        self.toolButtonHome.clicked.connect(self.effectOf_toolButtonHome)
        self.toolButtonCP.clicked.connect(self.effectOf_toolButtonCP)
        self.toolButtonHP.clicked.connect(self.effectOf_toolButtonHP)
        self.toolButtonTS.clicked.connect(self.effectOf_toolButtonTS)
        self.toolButtonMS.clicked.connect(self.effectOf_toolButtonMS)
        self.toolButtonArt.clicked.connect(self.effectOf_toolButtonArt)
        
        
        
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
        
        self.pushButtonCPMTOrbitsSinglePoint.clicked.connect(self.effectOf_pushButtonCPMTOrbitsSinglePoint) ### PERSONAL NOTE: this effect has not been fully/satisfactorily programmed
        self.pushButtonCPMTOrbitsRandomCircle.clicked.connect(self.effectOf_pushButtonCPMTOrbitsRandomCircle)
        self.pushButtonCPMTOrbitsSteinerGrid.clicked.connect(self.effectOf_pushButtonCPMTOrbitsSteinerGrid)

        self.pushButtonUHPClearCanvas.clicked.connect(self.effectOf_pushButtonUHPClearCanvas)
        self.proxyUHP = pg.SignalProxy(self.PlotWidgetIn_pageUHP.scene().sigMouseMoved, rateLimit=60, slot=self.UHPmouseMoved)
        self.UHPdraggableDots.Dot.moved.connect(self.UHPBCDragGeodesicSegment)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPBCGeodesicSegmentStatic)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPBCConvexHull)
        self.proxyUHPGM = pg.SignalProxy(self.PlotWidgetIn_pageUHPGeodesicMotion.scene().sigMouseMoved, rateLimit=60, slot=self.UHPGMmouseMoved)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPGMGeodesicSegmentAnimated)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPGMGeodesicRayConstantRapidityAnimated)
        self.PlotWidgetIn_pageUHP.scene().sigMouseClicked.connect(self.UHPGMWave)
        self.pushButtonUHPCMCircMotionAntiClockwise.clicked.connect(self.effectOf_pushButtonUHPCMCircMotionAntiClockwise)


        self.pushButtonPDClearCanvas.clicked.connect(self.effectOf_pushButtonPDClearCanvas)        
#        self.proxyPD = pg.SignalProxy(self.circROIPD.sigHoverEvent, rateLimit=60, slot=self.PDmouseMoved)
        self.proxyPD = pg.SignalProxy(self.PlotWidgetIn_pagePD.scene().sigMouseMoved, rateLimit=100, slot=self.PDmouseMoved)
        self.PlotWidgetIn_pagePD.scene().sigMouseClicked.connect(self.PDBCGeodesicSegmentStatic)
        self.PlotWidgetIn_pagePD.scene().sigMouseClicked.connect(self.PDGMGeodesicSegmentAnimated)










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
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen="w")
            else:
                finitePoint = removeooFromArgs(P,Q)[0]
                theta = numpy.linspace(0,360,n)
                for t in theta: ### triple comes in the format [[R.real,R.imag],P], where R and P are (finite) complex numbers. PERSONAL NOTE: This is NOT elegant!!!!!!
                    line = pg.InfiniteLine(pos = [finitePoint.real,finitePoint.imag], angle = t, pen='w')
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
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen ="w")
                centersAndRadii = Steiner_grids_CP.Apollonius().Apollonius_e_circles2(
                        P,Q,n)
                theta = numpy.linspace(0,1,101)
                for triple in centersAndRadii: ### triple comes in the format [(x,y),r]
                    x_coord = (triple[0])[0] + (triple[1])*numpy.cos(theta*2*numpy.pi)
                    y_coord = (triple[0])[1] + (triple[1])*numpy.sin(theta*2*numpy.pi)
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen="w")
            else:
                finitePoint = removeooFromArgs(P,Q)[0]
                theta = numpy.linspace(0,1,101)
                for t in range(1,(n)**2+1,n):
                    x_coord = finitePoint.real + t*numpy.cos(theta*2*numpy.pi)
                    y_coord = finitePoint.imag + t*numpy.sin(theta*2*numpy.pi)
                    self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen="w")
        except:
            pass

##############
############## 
#############################################
                
    def effectOf_pushButtonCPSGSteiner(self):
        self.effectOf_pushButtonCPSGCommon()
        self.effectOf_pushButtonCPSGApollonius()
        x = numpy.arange(0,10,1)
        y = x**2
        point = pg.ScatterPlotItem(x,y)
        self.PlotWidgetIn_pageCP.addItem(point)
        
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
                self.PlotWidgetIn_pageCP.plot(coord[0],coord[1],pen='w')
        except:
            pass
        
#############################################
##############
############## Something strange happens when one twoClicks on "Clear Canvas" and then
############## tries to plot the orbit of a new point   

    def effectOf_pushButtonCPMTOrbitsSinglePoint(self):
        MobiusTrans = Mobius_CP.MobiusAssocToMatrix(
                self.lineEditCPMTOrbitsComplexNumberalpha.text(),
                self.lineEditCPMTOrbitsComplexNumberbeta.text(),
                self.lineEditCPMTOrbitsComplexNumbergamma.text(),
                self.lineEditCPMTOrbitsComplexNumberdelta.text())
        z_0 = extendedValue(self.lineEditCPMTOrbitsComplexNumberz_0.text())
        numberOfPointsInOrbit = int(self.spinBoxCPMTOrbits.cleanText())
        Orbit = MobiusTrans.Mob_trans_iterable(z_0,numberOfPointsInOrbit)

        if z_0 != oo:
            self.radioButtonCPoo.setChecked(False)
            initialDot = pg.ScatterPlotItem([z_0.real],[z_0.imag],pen='w',brush='b')
            self.PlotWidgetIn_pageCP.addItem(initialDot)
        else:
            self.radioButtonCPoo.setChecked(True)
            
        k=0
        def update():
            self.radioButtonCPoo.setChecked(False)
            nonlocal k
            previous_z = Orbit[k]
            if previous_z != oo:
                previousDot = pg.ScatterPlotItem([previous_z.real],[previous_z.imag],pen='r')
                self.PlotWidgetIn_pageCP.addItem(previousDot)
            current_z = Orbit[(k+1)%numberOfPointsInOrbit]
            if current_z !=oo:
                currentDot = pg.ScatterPlotItem([current_z.real],[current_z.imag],pen='w',brush='b')
                self.PlotWidgetIn_pageCP.addItem(currentDot)
            else:
                self.radioButtonCPoo.setChecked(True)
            k = (k+1)%numberOfPointsInOrbit
        #    QtCore.QTimer.singleShot(1000, update)
        #update()
        
#        timer = QtCore.QTimer(self)
#        timer.timeout.connect(update)
#        timer.start(250)
        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(update)
        self.timer.start(250)
        
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
#            self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen="b")
        x_coord = ((centersAndRadii[0])[0])[0] + (centersAndRadii[0])[1]*numpy.cos(t)
        y_coord = ((centersAndRadii[0])[0])[1] + (centersAndRadii[0])[1]*numpy.sin(t)
        self.PlotWidgetIn_pageCP.plot(x_coord,y_coord,pen="w")
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
            self.PlotWidgetIn_pageCP.plot(X_next,Y_next,pen='w',clear=False)
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
                #self.PlottingCircleOrLineThrough3Points(P,previous_Pt2,Q,'b')
                previous_points = pg.ScatterPlotItem([previous_Pt1.real,previous_Pt2.real,previous_Pt3.real],[previous_Pt1.imag,previous_Pt2.imag,previous_Pt3.imag],pen='b')
                self.PlotWidgetIn_pageCP.addItem(previous_points)
                current_Pt1 = Orbit1[(k+1)%nIterations]
                current_Pt2 = Orbit2[(k+1)%nIterations]
                current_Pt3 = Orbit3[(k+1)%nIterations]
                #self.PlottingCircleOrLineThrough3Points(current_Pt1,current_Pt2,current_Pt3,'w')
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
            
            self.PlottingCircleOrLineThrough3Points(Pt1OnMdx,Pt2OnMdx,Pt3OnMdx,'w')
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
                self.PlottingCircleOrLineThrough3Points(P,previous_Pt2,Q,'b')
                previous_point = pg.ScatterPlotItem([previous_Pt2.real],[previous_Pt2.imag],pen='b')
                self.PlotWidgetIn_pageCP.addItem(previous_point)
                current_Pt1 = Orbit1[(k+1)%nIterations]
                current_Pt2 = Orbit2[(k+1)%nIterations]
                current_Pt3 = Orbit3[(k+1)%nIterations]
                self.PlottingCircleOrLineThrough3Points(current_Pt1,current_Pt2,current_Pt3,'w')
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
            
            self.PlottingCircleOrLineThrough3Points(finitePoint+1,finitePoint+1j,finitePoint-1,'w')
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
                self.PlottingCircleOrLineThrough3Points(P,previous_Pt2,Q,'b')
                previous_point = pg.ScatterPlotItem([previous_Pt2.real],[previous_Pt2.imag],pen='b')
                self.PlotWidgetIn_pageCP.addItem(previous_point)
                current_Pt1 = Orbit1[(k+1)%nIterations]
                current_Pt2 = Orbit2[(k+1)%nIterations]
                current_Pt3 = Orbit3[(k+1)%nIterations]
                self.PlottingCircleOrLineThrough3Points(current_Pt1,current_Pt2,current_Pt3,'w')
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
        self.UHPdraggableDots.setData(pos=numpy.array([[0,-100]]))
        self.PlotWidgetIn_pageUHP.addItem(self.UHPdraggableDots)
        self.PlotWidgetIn_pageUHP.addItem(self.boundingLineUHP)
        self.boundingLineUHP.setPen(pg.mkPen('g', width=self.widthBoundingLineUHP))
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
            self.boundingLineUHP.setPen(pg.mkPen('g', width=self.widthBoundingLineUHP))
            self.labelUHPxNumber.setText("<span style='font-size: 12pt'><span style='color: green'>x=%0.100f" % (mousePoint.x()))
            self.labelUHPyNumber.setText("<span style='font-size: 12pt'><span style='color: green'>y=%0.100f" % (mousePoint.y()))
        if mousePoint.y() == 0:
            self.boundingLineUHP.setPen(pg.mkPen('b', width=self.widthBoundingLineUHP))
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
            global twoClicks
            global arbManyClicks
            global auxStorage
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y < 0:
                pass
            else:
                while len(twoClicks) < 3:  
                    twoClicks.append([x,y])
                if len(twoClicks) == 3:  
                    del twoClicks[0]
                #print(twoClicks)
                if twoClicks[0] == twoClicks[1]:
                    arbManyClicks = numpy.array([[twoClicks[0][0],twoClicks[0][1]]],dtype=float)
                    #initialPoint = pg.GraphItem(pos=[[twoClicks[0][0],twoClicks[0][1]]])
                    self.lineEditUHPGMComplexNumber1.setText(str(twoClicks[0][0]+twoClicks[0][1]*(1j)))
                    self.UHPdraggableDots.setData(pos=arbManyClicks,  pxMode=True)
                else:
                    #print(twoClicks)
                    arbManyClicks = numpy.concatenate((arbManyClicks,numpy.array([[twoClicks[1][0],twoClicks[1][1]]],dtype=float)))
                    self.UHPdraggableDots.setData(pos=arbManyClicks,  pxMode=True)
                    #self.PlotWidgetIn_pageUHP.addItem(finalPoint)
                    P = twoClicks[0][0]+twoClicks[0][1]*(1j)
                    Q = twoClicks[1][0]+twoClicks[1][1]*(1j)
                    self.lineEditUHPGMComplexNumber2.setText(str(Q))
                    #print(P,Q)
                    geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                    drawing = pg.PlotCurveItem(x_coord,y_coord,pen='w')
                    self.PlotWidgetIn_pageUHP.addItem(drawing)
                    auxStorage.append(drawing)
                    #print(arbManyClicks)
#                    print(auxStorage)
                    self.labelUHPBChdistancenumber.setNum(UHP_HP.UHPBasics().UHPDist(P,Q))
                    
#        if self.checkBoxUHPEnableClickOnCanvas.isChecked() == False:
#            for clicked in Clicks:
#                clicked.clear()
            #print("as expected")        
    @QtCore.pyqtSlot(object,int)
    def UHPBCDragGeodesicSegment(self,pt,ind):
        global arbManyClicks
        global auxStorage
        if len(arbManyClicks) < 2:
            pass
        else:
            P = pt[0]+pt[1]*(1j)
            if ind == 0:
                neighbour = arbManyClicks[ind+1]
                Q = neighbour[0]+neighbour[1]*(1j)
                curve = auxStorage[ind]
                geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
            elif ind == len(arbManyClicks)-1:
                neighbour = arbManyClicks[ind-1]
                Q = neighbour[0]+neighbour[1]*(1j)
                curve = auxStorage[ind-1]
                geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(P,Q)
                x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                curve.setData(x_coord,y_coord)
            else:
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
                
                
                
                
            
                
            
                
    
    def UHPBCConvexHull(self,ev):
        if self.radioButtonUHPBCConvexHull.isChecked() == True and self.stackedWidgetIn_pageUHP.currentIndex() == 0:
            global arbManyClicks
            global auxStorage
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y < 0:
                pass
            else:
                currentPoint = pg.ScatterPlotItem([x],[y])
                self.PlotWidgetIn_pageUHP.addItem(currentPoint)
                arbManyClicks.append(x+y*(1j))
                if len(arbManyClicks) < 2:
                    auxStorage.append(x+y*(1j))
#                    X, Y = arbManyClicks[0].real, arbManyClicks[0].imag
#                    firstPoint = pg.ScatterPlotItem([X],[Y])
#                    self.PlotWidgetIn_pageUHP.addItem(firstPoint)
                elif len(arbManyClicks) == 2:
                    auxStorage.append(x+y*(1j))
#                    X, Y = arbManyClicks[1].real, arbManyClicks[1].imag
#                    secondPoint = pg.ScatterPlotItem([X],[Y])
#                    self.PlotWidgetIn_pageUHP.addItem(secondPoint)
                    geodesicSeg = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(arbManyClicks[0],arbManyClicks[1])
                    x_coord, y_coord = geodesicSeg.real, geodesicSeg.imag
                    self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='w')
                else:
                    self.PlotWidgetIn_pageUHP.clear()
                    self.PlotWidgetIn_pageUHP.addItem(self.boundingLineUHP)
                    self.boundingLineUHP.setPen(pg.mkPen('g', width=self.widthBoundingLineUHP))
                    for p in arbManyClicks:
                        X, Y = p.real, p.imag
                        Point = pg.ScatterPlotItem([X],[Y])
                        self.PlotWidgetIn_pageUHP.addItem(Point)
                    if UHP_HP.UHPBasics().is_point_in_h_convex_hull(x+y*(1j),auxStorage) == False:
                        auxStorage.append(x+y*(1j))
                    vertices = UHP_HP.UHPBasics().verts_h_polygon_counter_clockwise(auxStorage)
                    for i in range(0,len(vertices)-1,1):
                        geodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(vertices[i],vertices[i+1])
                        x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                        self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='w')
                        vertex = pg.ScatterPlotItem([vertices[i].real],[vertices[i].imag], pen = 'r', brush = 'r')
                        self.PlotWidgetIn_pageUHP.addItem(vertex)
                    auxStorage.clear()
                    auxStorage.extend(vertices)
#        if self.checkBoxUHPEnableClickOnCanvas.isChecked() == False:
#            for clicked in Clicks:
#                clicked.clear()
                
                
                
            
            
    


        
            
        
            

        
    def UHPGMGeodesicSegmentAnimated(self,ev):
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
                    plottedCurve = self.PlotWidgetIn_pageUHP.plot(pen = 'w')
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
#                            #self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='w')   
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
                            #self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='w')   
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



    def UHPGMGeodesicRayConstantRapidityAnimated(self,ev):
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
    #                    Line = pg.InfiniteLine(pos=u, angle=90, pen=pg.mkPen('g', width=self.widthBoundingLineUHP))
    #                    self.PlotWidgetIn_pageUHPGeodesicMotion.addItem(Line)
                        initialVector = u + v*(1j)
                        parametrization = UHP_HP.UHPGeodesicMotion().UHPGeodesicSegmentConstantRapidity(startPoint,initialVector)
                        numberOfSteps = 250
                        k = 0
                        def update():
                            nonlocal k
                            if k < numberOfSteps:
                                partialInterval = numpy.linspace(k,k+1)
                                x_coord = parametrization(partialInterval).real
                                y_coord = parametrization(partialInterval).imag
                                self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='w')
                                #PointBlue = pg.ScatterPlotItem([x_coord[-1]],[y_coord[-1]],pen='b',brush = 'b')
                                #self.PlotWidgetIn_pageUHP.addItem(PointBlue)
                                k = k+1
                            if k == numberOfSteps:
                                x_coord = parametrization(k+1).real
                                y_coord = parametrization(k+1).imag
                                finalPointRed = pg.ScatterPlotItem([x_coord],[y_coord],pen='r',brush = 'r')
                                self.PlotWidgetIn_pageUHP.addItem(finalPointRed)
                            QtCore.QTimer.singleShot(10, update)
                        update()
                #self.PlotWidgetIn_pageUHPGeodesicMotion.show()
                if len(arbManyClicks) == 1:
                    self.PlotWidgetIn_pageUHPGeodesicMotion.scene().sigMouseClicked.connect(UHPGMGetVector)
                else:
                    pass
#                self.PlotWidgetIn_pageUHPGeodesicMotion.scene().sigMouseClicked.disconnect(UHPGMGetVector)
                
    def UHPGMWave(self,ev):
        if self.radioButtonUHPGMWaves.isChecked() == True:
            global arbManyClicks
            x = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pageUHP.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if y <= 0:
                pass
            else:
                arbManyClicks.append(x + y*(1j))
                Point = pg.ScatterPlotItem([x],[y],pen='r',brush = 'r')
                self.PlotWidgetIn_pageUHP.addItem(Point)
                rapidity = int(self.spinBoxUHPGMWaveRapidity.cleanText())
                numberOfSteps = 5
                k = 0
                def update():
                    nonlocal k
                    if k < numberOfSteps:
                        eCenterAndRadius = UHP_HP.UHPBasics().eCenterAndRadiusH_Circle(x + y*(1j),k*rapidity)
                        eCenter = eCenterAndRadius[0]
                        eRadius = eCenterAndRadius[1]
                        circle = pg.PlotCurveItem(eCenter.real+eRadius*numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),eCenter.imag+eRadius*numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen='w')
                        self.PlotWidgetIn_pageUHP.addItem(circle)
                        k = k+1
#                    if k == numberOfSteps:
#                        eCentAndRad = UHP_HP.UHPBasics().eCenterAndRadiusH_Circle(x + y*(1j),k*rapidity)
#                        eCent = eCentAndRad[0]
#                        eRad = eCentAndRad[1]
#                        circ = pg.PlotCurveItem(eCent.real+eRad*numpy.cos(numpy.linspace(0,2*numpy.pi,1000)),eCent.imag+eRad*numpy.sin(numpy.linspace(0,2*numpy.pi,1000)), pen='r')
#                        self.PlotWidgetIn_pageUHP.addItem(circ)
#                        k = k+1
                    QtCore.QTimer.singleShot(1000, update)
                update()
                            
                #eCenterAndRadius = UHP_HP.UHPBasics().eCenterAndRadiusH_Circle()
            

            
        
            
    def effectOf_pushButtonUHPCMCircMotionAntiClockwise(self):
        curve = UHP_HP.UHPCircularMotion().UHPCircSegmentParamByArcLength(200j,150,-numpy.pi,numpy.pi)[0]
        length = UHP_HP.UHPCircularMotion().UHPCircSegmentParamByArcLength(200j,150,-numpy.pi,numpy.pi)[1]
        numberOfSteps = 500
        t = numpy.linspace(0,5*length,numberOfSteps+1)
        hcenter = numpy.sqrt(200**2-150**2)*(1j)
        k=0
        def update():
            nonlocal k
            if k < len(t)-1:
                partialInterval = numpy.linspace(t[k],t[k+1],100)
                x_coord = curve(partialInterval).real
                y_coord = curve(partialInterval).imag
                self.PlotWidgetIn_pageUHP.plot(x_coord,y_coord,pen='w')
                currentPt= curve(t[k])
                radialGeodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(hcenter,currentPt)
                self.PlotWidgetIn_pageUHP.plot(radialGeodesicSegment.real,radialGeodesicSegment.imag,pen='r')
                nextPt= curve(t[k+1])
                radialGeodesicSegment = UHP_HP.UHPBasics().UHPGeodesicSegment_rcostrsint(hcenter,nextPt)
                self.PlotWidgetIn_pageUHP.plot(radialGeodesicSegment.real,radialGeodesicSegment.imag,pen='w')
            k = (k+1)
            QtCore.QTimer.singleShot(1000*numpy.ceil(numpy.abs(length))/numberOfSteps, update)
        update()
        
            
            
#####
#####






























#################################
#################################
#################################
### HYPERBOLIC PLANE
### UPPER HALF PLANE --- PD

    def effectOf_pushButtonPDClearCanvas(self):
        self.PlotWidgetIn_pagePD.clear()
        self.PlotWidgetIn_pagePD.addItem(self.boundingCirclePD)
        self.boundingCirclePD.setPen("g")
        for clicked in Clicks:
            clicked.clear()
        

    def PDmouseMoved(self,evt):
        pos = evt[0]  ## using signal proxy turns original arguments into a tuple
#        if self.PlotWidgetIn_pageUHP.sceneBoundingRect().contains(pos):
        mousePoint = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(pos)
        #index = int(mousePoint.x())
        if mousePoint.x()**2+mousePoint.y()**2 < 1:
            self.boundingCirclePD.setPen("g")
            self.labelPDxNumber.setText("<span style='font-size: 12pt'><span style='color: green'>x=%0.100f" % (mousePoint.x()))
            self.labelPDyNumber.setText("<span style='font-size: 12pt'><span style='color: green'>y=%0.100f" % (mousePoint.y()))
            self.labelPDNormNumber.setText("<span style='font-size: 12pt'><span style='color: green'>Norm=%0.100f" % (mousePoint.x()**2+mousePoint.y()**2))

        #index = int(mousePoint.x())
        if mousePoint.x()**2+mousePoint.y()**2 == 1:
            self.boundingCirclePD.setPen("b")
            self.labelPDxNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>x=%0.100f" % (mousePoint.x()))
            self.labelPDyNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>y=%0.100f" % (mousePoint.y()))
            self.labelPDNormNumber.setText("<span style='font-size: 12pt'><span style='color: blue'>Norm=%0.100f" % (mousePoint.x()**2+mousePoint.y()**2))
        if mousePoint.x()**2+mousePoint.y()**2 > 1:
            self.boundingCirclePD.setPen("r")
            self.labelPDxNumber.setText("<span style='font-size: 12pt'><span style='color: red'>--")
            self.labelPDyNumber.setText("<span style='font-size: 12pt'><span style='color: red'>--")
            self.labelPDNormNumber.setText("<span style='font-size: 12pt'><span style='color: red'>--")

#        print(mousePoint.x())
#        print(mousePoint.y())
#        print("yes")



    def PDBCGeodesicSegmentStatic(self,ev):
        if self.checkBoxPDEnableClickOnCanvas.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 0:
            global twoClicks
            x = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).x()
            y = self.PlotWidgetIn_pagePD.plotItem.vb.mapSceneToView(ev.scenePos()).y()
            if x**2 + y**2 > 1:
                pass
            else:
                while len(twoClicks) < 3:  
                    twoClicks.append([x,y])
                #print(twoClicks)
                if len(twoClicks) == 3:  
                    del twoClicks[0]
                #print(twoClicks)
                if twoClicks[0] == twoClicks[1]:
                    initialPoint = pg.ScatterPlotItem([twoClicks[0][0]],[twoClicks[0][1]])
                    self.PlotWidgetIn_pagePD.addItem(initialPoint)
                    self.lineEditPDBCComplexNumber1.setText(str(twoClicks[0][0]+twoClicks[0][1]*(1j)))
                else:
                    #print(twoClicks)
                    finalPoint = pg.ScatterPlotItem([twoClicks[1][0]],[twoClicks[1][1]])
                    self.PlotWidgetIn_pagePD.addItem(finalPoint)
                    P = twoClicks[0][0]+twoClicks[0][1]*(1j)
                    Q = twoClicks[1][0]+twoClicks[1][1]*(1j)
                    self.lineEditPDBCComplexNumber2.setText(str(Q))
                    #print(P,Q)
                    geodesicSegment = PD_HP.PDBasics().PDGeodesicSegment_rcostrsint(P,Q)
                    x_coord, y_coord = geodesicSegment.real, geodesicSegment.imag
                    self.PlotWidgetIn_pagePD.plot(x_coord,y_coord,pen='w')
                    self.labelPDBChdistancenumber.setNum(PD_HP.PDBasics().PDDist(P,Q))
        if self.checkBoxPDEnableClickOnCanvas.isChecked() == False:
            twoClicks.clear()
            print("as expected")        
    
    def PDGMGeodesicSegmentAnimated(self,ev):
        if self.checkBoxPDEnableClickOnCanvas.isChecked() == True and self.stackedWidgetIn_pagePD.currentIndex() == 1:
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
                    plottedCurve = self.PlotWidgetIn_pagePD.plot(pen = 'w')
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
                        QtCore.QTimer.singleShot(1000*numpy.ceil(s)/numberOfSteps, update)
                    update()
                #print(numpy.ceil(s))
                
        #        timer = QtCore.QTimer(self)
        #        timer.timeout.connect(update)
        #        timer.start(250)
#                self.timer = QtCore.QTimer(self)
#                self.timer.timeout.connect(update)
#                self.timer.start(1)
                
        if self.checkBoxPDEnableClickOnCanvas.isChecked() == False:
            twoClicks.clear()
            #print("as expected")
       

        
        
        
app = QtWidgets.QApplication(sys.argv)
form = appMainWindow()
form.show()
app.exec_()

#if __name__ == "__main__":            
#    app = QtWidgets.QApplication(sys.argv)
#    form = appMainWindow()
#    form.show()
#    app.exec_()