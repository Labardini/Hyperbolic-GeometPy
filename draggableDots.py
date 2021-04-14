#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  4 18:46:31 2018

@author: daniellabardini
"""

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import numpy




class DotDragSignal(QtCore.QObject):
    moved = QtCore.pyqtSignal(object,int)

    def __init__(self, pt, ind):
        QtCore.QObject.__init__(self)
        self._pt = pt
        self._ind = ind

    @property
    def pt(self):
        return self._pt
    
    @property
    def ind(self):
        return self._ind

    
    @pt.setter
    def pt(self, new_pt):
        self._pt = new_pt
        self.moved.emit(new_pt,self.ind)
        
    @ind.setter
    def ind(self, new_ind):
        self._ind = new_ind





class draggableDot(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.Dot = DotDragSignal([0,0],-1)
#        self.textItems = []
        pg.GraphItem.__init__(self)
        #self.scatter.sigClicked.connect(self.clicked)
        
    def setData(self, **kwds):
        #self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = numpy.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = numpy.arange(npts)
        #self.setTexts(self.text)
        self.updateGraph()
        
#    def setTexts(self, text):
#        for i in self.textItems:
#            i.scene().removeItem(i)
#        self.textItems = []
#        for t in text:
#            item = pg.TextItem(t)
#            self.textItems.append(item)
#            item.setParentItem(self)
        
    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        
        
#        for i,item in enumerate(self.textItems):
#            item.setPos(*self.data['pos'][i])
#        
        
    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        
        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first 
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.Dot.ind = ind
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return
        
        ind = self.dragPoint.data()[0]
        self.data['pos'][ind] = ev.pos() + self.dragOffset
        self.Dot.pt = self.data['pos'][ind]
        self.updateGraph()
        ev.accept()
        
#    def clicked(self, pts):
#        print("clicked: %s" % pts)



















class unitCircledraggableDot(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.Dot = DotDragSignal([0,0],-1)
#        self.textItems = []
        pg.GraphItem.__init__(self)
        #self.scatter.sigClicked.connect(self.clicked)
        
    def setData(self, **kwds):
        #self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = numpy.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = numpy.arange(npts)
        #self.setTexts(self.text)
        self.updateGraph()
        
#    def setTexts(self, text):
#        for i in self.textItems:
#            i.scene().removeItem(i)
#        self.textItems = []
#        for t in text:
#            item = pg.TextItem(t)
#            self.textItems.append(item)
#            item.setParentItem(self)
        
    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        
        
#        for i,item in enumerate(self.textItems):
#            item.setPos(*self.data['pos'][i])
#        
        
    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        
        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first 
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.Dot.ind = ind
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return
        
        ind = self.dragPoint.data()[0]
        a, b = (ev.pos() + self.dragOffset)[0], (ev.pos() + self.dragOffset)[1]
        if a**2+b**2 == 1 or a**2+b**2==0:
            self.data['pos'][ind] = ev.pos() + self.dragOffset
        if a**2+b**2 != 1:
            #alpha = 1.0000000001
            self.data['pos'][ind] = (ev.pos() + self.dragOffset)/ (numpy.sqrt(a**2+b**2))#*alpha)
        self.Dot.pt = self.data['pos'][ind]
        self.updateGraph()
        ev.accept()
        
#    def clicked(self, pts):
#        print("clicked: %s" % pts)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
class PosRealsdraggableDot(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.Dot = DotDragSignal([0,0],-1)
#        self.textItems = []
        pg.GraphItem.__init__(self)
        #self.scatter.sigClicked.connect(self.clicked)
        
    def setData(self, **kwds):
        #self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = numpy.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = numpy.arange(npts)
        #self.setTexts(self.text)
        self.updateGraph()
        
#    def setTexts(self, text):
#        for i in self.textItems:
#            i.scene().removeItem(i)
#        self.textItems = []
#        for t in text:
#            item = pg.TextItem(t)
#            self.textItems.append(item)
#            item.setParentItem(self)
        
    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        
        
#        for i,item in enumerate(self.textItems):
#            item.setPos(*self.data['pos'][i])
#        
        
    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        
        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first 
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.Dot.ind = ind
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return
        
        ind = self.dragPoint.data()[0]
        a, b = (ev.pos() + self.dragOffset)[0], (ev.pos() + self.dragOffset)[1]
        if a > 0:
            self.data['pos'][ind][0] = a
            self.data['pos'][ind][1] = 0
        if a <= 0:
            alpha = 1.0000000001
            self.data['pos'][ind][0] = alpha
            self.data['pos'][ind][1] = 0
        self.Dot.pt = self.data['pos'][ind]
        self.updateGraph()
        ev.accept()
        
#    def clicked(self, pts):
#        print("clicked: %s" % pts)























class UHPdraggableDot(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.Dot = DotDragSignal([0,0],-1)
#        self.textItems = []
        pg.GraphItem.__init__(self)
        #self.scatter.sigClicked.connect(self.clicked)
        
    def setData(self, **kwds):
        #self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = numpy.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = numpy.arange(npts)
        #self.setTexts(self.text)
        self.updateGraph()
        
#    def setTexts(self, text):
#        for i in self.textItems:
#            i.scene().removeItem(i)
#        self.textItems = []
#        for t in text:
#            item = pg.TextItem(t)
#            self.textItems.append(item)
#            item.setParentItem(self)
        
    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        
        
#        for i,item in enumerate(self.textItems):
#            item.setPos(*self.data['pos'][i])
#        
        
    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        
        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first 
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.Dot.ind = ind
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return
        
        ind = self.dragPoint.data()[0]
        a, b = (ev.pos() + self.dragOffset)[0], (ev.pos() + self.dragOffset)[1]
        if b >= 0:
            self.data['pos'][ind] = ev.pos() + self.dragOffset
        if b < 0:
            self.data['pos'][ind] = (a,0)
        self.Dot.pt = self.data['pos'][ind]
        self.updateGraph()
        ev.accept()
        
#    def clicked(self, pts):
#        print("clicked: %s" % pts)

















class PDdraggableDot(pg.GraphItem):
    def __init__(self):
        self.dragPoint = None
        self.dragOffset = None
        self.Dot = DotDragSignal([0,0],-1)
#        self.textItems = []
        pg.GraphItem.__init__(self)
        #self.scatter.sigClicked.connect(self.clicked)
        
    def setData(self, **kwds):
        #self.text = kwds.pop('text', [])
        self.data = kwds
        if 'pos' in self.data:
            npts = self.data['pos'].shape[0]
            self.data['data'] = numpy.empty(npts, dtype=[('index', int)])
            self.data['data']['index'] = numpy.arange(npts)
        #self.setTexts(self.text)
        self.updateGraph()
        
#    def setTexts(self, text):
#        for i in self.textItems:
#            i.scene().removeItem(i)
#        self.textItems = []
#        for t in text:
#            item = pg.TextItem(t)
#            self.textItems.append(item)
#            item.setParentItem(self)
        
    def updateGraph(self):
        pg.GraphItem.setData(self, **self.data)
        
        
#        for i,item in enumerate(self.textItems):
#            item.setPos(*self.data['pos'][i])
#        
        
    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        
        if ev.isStart():
            # We are already one step into the drag.
            # Find the point(s) at the mouse cursor when the button was first 
            # pressed:
            pos = ev.buttonDownPos()
            pts = self.scatter.pointsAt(pos)
            if len(pts) == 0:
                ev.ignore()
                return
            self.dragPoint = pts[0]
            ind = pts[0].data()[0]
            self.Dot.ind = ind
            self.dragOffset = self.data['pos'][ind] - pos
        elif ev.isFinish():
            self.dragPoint = None
            return
        else:
            if self.dragPoint is None:
                ev.ignore()
                return
        
        ind = self.dragPoint.data()[0]
        a, b = (ev.pos() + self.dragOffset)[0], (ev.pos() + self.dragOffset)[1]
        if a**2+b**2 < 1:
            self.data['pos'][ind] = ev.pos() + self.dragOffset
        if a**2+b**2 >= 1:#PERSONAL NOTE: notice the factor alpha IT IS NOT GOOD PROGRAMMING OR MATHEMATICAL PRACTICE, BUT WITHOUT THE FACTOR alpha THERE WAS AN UGLY PROBLEM THAT I BELIEVE CAME FROM A FLOATING POINT ISSUE
            alpha = 1.0000000001
            self.data['pos'][ind] = (ev.pos() + self.dragOffset)/ ((numpy.sqrt(a**2+b**2))*alpha)
        self.Dot.pt = self.data['pos'][ind]
        self.updateGraph()
        ev.accept()
        
#    def clicked(self, pts):
#        print("clicked: %s" % pts)



