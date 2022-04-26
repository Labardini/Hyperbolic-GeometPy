#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 17 17:07:25 2017

@author: daniellabardini
"""

import numpy

import pyqtgraph as pg

from exception_handling import myInputError
from Maths.CP_Maths import extended_complex_plane_CP
from Maths.CP_Maths import Steiner_grids_CP


#### SOME FUNCTIONS IMPORTED FROM Maths.CP_Maths.extended_complex_plane_CP
oo = extended_complex_plane_CP.numpyExtendedComplexPlane().oo
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




#############################

class AnalyticContinuation:
    
    def __init__(self):
        pass
    
    def nthRoots(self,n):
        def allnthRoots(z):
            float1 = float(1)
            floatn = float(n)
            specificnthRoot = numpy.power(z, (float1/floatn))
            nthRootOfUnity = numpy.cos(float(2)*float(numpy.pi)/floatn) + (numpy.sin(float(2)*float(numpy.pi)/floatn))*(1j)
            listOfAllnthRoots = [(nthRootOfUnity**float(k))*specificnthRoot for k in range(n)]
            return listOfAllnthRoots
        return allnthRoots
    

                
    def mediatricesBetweenConsecutiventhRoots(self,n):
        def mediatricesBetweennthRoots(z):
            listOfAllnthRoots = self.nthRoots(n)(z)
            mediatrices = []
            floatn = float(n)
            for k in range(n):
                mediatrix = listOfAllnthRoots[k]*(numpy.cos(float(numpy.pi)/floatn) + (numpy.sin(float(numpy.pi)/floatn))*(1j))
                mediatrices.append(mediatrix)
            return mediatrices
        return mediatricesBetweennthRoots
    
    def slitRayForGermOfnthRoot(self,n):
        def slitRay(z):
            float1 = float(1)
            floatn = float(n)
            specificnthRoot = numpy.power(z, (float1/floatn))
            
            ptOnSlitRay = (specificnthRoot*(numpy.cos(float(numpy.pi)/floatn) + (numpy.sin(float(numpy.pi)/floatn))*(1j)))**n
            #angleInDegrees = (180*(numpy.angle(ptOnSlitRay)/numpy.pi))+90
            #pgLineInfo = {"pos" : position, "angle" : angleInDegrees}
            #dictionary = {"ptOnSlitRay" : ptOnSlitRay}#, "pgLineInfo" : pgLineInfo}
            return ptOnSlitRay#dictionary
        return slitRay
            
            
            
        
        
            
            