#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Created on Sat May 15 12:42:21 2021

@author: caesar
'''


from PyQt5 import QtGui
import pyqtgraph as pg

class choosingBackground:
    
    def __init__(self,backgroundColor="Black"):
        
        if backgroundColor == 'Black':
            self.black = QtGui.QColor(0,0,0) # NATURAL BLACK
            self.white = QtGui.QColor(255,255,255) # NATURAL WHITE
            self.blue = QtGui.QColor(0,0,255) # NATURAL BLUE
            self.red = QtGui.QColor(255,0,0) # NATURAL RED
            self.cyan = QtGui.QColor(0,255,255) # NATURAL CYAN
            self.yellow = QtGui.QColor(255,255,0) # NATURAL YELLOW
            self.magenta = QtGui.QColor(255,0,255) # NATURAL MAGENTA
            self.green = QtGui.QColor(0,255,0) # NATURAL GREEN
            self.backgroundBrush = QtGui.QBrush(QtGui.QColor(0,0,0))
            
            
        if backgroundColor == 'White':
            self.black = QtGui.QColor(255,255,255) # NATURAL WHITE
            self.white = QtGui.QColor(0,0,0) # NATURAL BLACK
            self.blue = QtGui.QColor(255,0,0) # NATURAL RED
            self.red = QtGui.QColor(0,0,255) # NATURAL BLUE
            self.cyan = QtGui.QColor(255,0,255) # NATURAL MAGENTA
            self.yellow = QtGui.QColor(0,255,0) # NATURAL GREEN
            self.magenta = QtGui.QColor(0,255,255) # NATURAL CYAN
            self.green = QtGui.QColor(255,255,0) # NATURAL YELLOW
            self.backgroundBrush = QtGui.QBrush(QtGui.QColor(255,255,255))
        
        if backgroundColor == 'Blue':
            self.black = QtGui.QColor(0,0,255) # NATURAL BLUE
            self.white = QtGui.QColor(255,255,255) # NATURAL WHITE
            self.blue = QtGui.QColor(255,0,0) # NATURAL RED
            self.red = QtGui.QColor(255,0,255) # NATURAL MAGENTA
            self.cyan = QtGui.QColor(0,0,0) # NATURAL BLACK
            self.yellow = QtGui.QColor(0,255,0) # NATURAL GREEN
            self.magenta = QtGui.QColor(0,255,255) # NATURAL CYAN
            self.green = QtGui.QColor(255,255,0) # NATURAL YELLOW
            self.backgroundBrush = QtGui.QBrush(QtGui.QColor(0,0,255))
            
        if backgroundColor == 'Red':
            self.black = QtGui.QColor(255,0,0) # NATURAL RED
            self.white = QtGui.QColor(255,255,255) # NATURAL WHITE
            self.blue = QtGui.QColor(0,0,255) # NATURAL BLUE
            self.red = QtGui.QColor(0,0,0) # NATURAL BLACK
            self.cyan = QtGui.QColor(255,0,255) # NATURAL MAGENTA
            self.yellow = QtGui.QColor(255,255,0) # NATURAL YELLOW
            self.magenta = QtGui.QColor(0,255,255) # NATURAL CYAN
            self.green = QtGui.QColor(0,255,0) # NATURAL GREEN
            self.backgroundBrush = QtGui.QBrush(QtGui.QColor(255,0,0))
            
            
            ### pens
        self.blackPenWidth2 = pg.mkPen(color=self.black, width=2)
        self.whitePenWidth2 = pg.mkPen(color=self.white, width=2)
        self.bluePenWidth2 = pg.mkPen(color=self.blue, width=2)
        self.redPenWidth2 = pg.mkPen(color=self.red, width=2)
        self.cyanPenWidth2 = pg.mkPen(color=self.cyan, width=2)
        self.yellowPenWidth2 = pg.mkPen(color=self.yellow, width=2)
        self.magentaPenWidth2 = pg.mkPen(color=self.magenta, width=2)
        self.greenPenWidth2 = pg.mkPen(color=self.green, width=2)

        
            
        
            
        
            
            
        
        
        
        
    




        