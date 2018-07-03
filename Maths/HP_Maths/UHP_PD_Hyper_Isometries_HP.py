#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 14 09:37:03 2018

@author: daniellabardini
"""

import numpy



from exception_handling import myInputError
from Maths.CP_Maths import extended_complex_plane_CP


#### SOME FUNCTIONS IMPORTED FROM Maths.CP_Maths.extended_complex_plane_CP
oo = extended_complex_plane_CP.numpyExtendedComplexPlane().oo
extendedValue = extended_complex_plane_CP.numpyExtendedComplexPlane().extendedValue
####

class CanonicalIsometriesBetweenModels:
    
    def __init__(self):    
        pass
    
    def IsometryUHPtoPD(self,point):
        P = extendedValue(point)
        if P != oo and P != -(1j):   
            pointInPD = ( ((1j)*P) + 1) / ( P + (1j) )
        if P == oo:
            pointInPD = 1j
        if P == -(1j):
            pointInPD = oo
        return pointInPD
    
    def IsometryPDtoUHP(self,point):
        P = extendedValue(point)
        if P != oo and P != 1j:   
            pointInUHP = ( ((1j)*P) -1 ) / ( -P + (1j) )
        if P == oo:
            pointInUHP = -(1j)
        if P == 1j:
            pointInUHP = oo
        return pointInUHP
    