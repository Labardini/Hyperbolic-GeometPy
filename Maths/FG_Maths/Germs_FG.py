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
    
    def mediatricesBetweenConsecutiveComplexNumbersInAList(self,n):
        def mediatrices(lista):
            result = []
            for k in range(len(lista)):
                midPoint = (lista[(k+1)%n]-lista[k%n])/2
                orthVector = (1j)*(lista[(k+1)%n]-lista[k%n])
                position = numpy.array([[midPoint.real,midPoint.imag]],dtype=float)
                angleInDegrees = (180*(numpy.angle((lista[(k+1)%n]-lista[k%n])/2)/numpy.pi))+90
                pgLineInfo = {"pos" : position, "angle" : angleInDegrees}
                dictionary = {"midPoint" : midPoint, "orthVector" : orthVector, "pgLineInfo" : pgLineInfo}
                result.append(dictionary)
            return result
        return mediatrices
                
                
    def mediatricesBetweenConsecutiventhRoots(self,n):
        def mediatricesBetweennthRoots(z):
            listOfAllnthRoots = self.nthRoots(n)(z)
            mediatrices = self.mediatricesBetweenConsecutiveComplexNumbersInAList(n)(listOfAllnthRoots)
            return mediatrices
        return mediatricesBetweennthRoots
    
    def slitRayForGermOfnthRoot(self,n):
        def slitRay(z):
            float1 = float(1)
            floatn = float(n)
            specificnthRoot = numpy.power(z, (float1/floatn))
            nthRootOfUnity = numpy.cos(float(2)*float(numpy.pi)/floatn) + (numpy.sin(float(2)*float(numpy.pi)/floatn))*(1j)
            midPoint = (nthRootOfUnity*specificnthRoot - specificnthRoot)/2
            ptOnSlitRay = midPoint**n
            position = numpy.array([[0,0]],dtype=float)
            angleInDegrees = (180*(numpy.angle(ptOnSlitRay)/numpy.pi))+90
            pgLineInfo = {"pos" : position, "angle" : angleInDegrees}
            dictionary = {"ptOnSlitRay" : ptOnSlitRay, "pgLineInfo" : pgLineInfo}
            return dictionary
        return slitRay
            
            
            
        
        
            
            