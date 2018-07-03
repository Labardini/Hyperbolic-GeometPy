#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 17 17:07:25 2017

@author: daniellabardini
"""

import numpy



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

class MobiusAssocToMatrix:
    
    def __init__(self,complexa,complexb,complexc,complexd):
        self.a = numpy.complex(complexa)
        self.b = numpy.complex(complexb)
        self.c = numpy.complex(complexc)
        self.d = numpy.complex(complexd)
        self.theMatrix = numpy.matrix([[self.a,self.b],[self.c,self.d]])
        self.theDet = self.a*self.d - self.b*self.c
        if self.theDet == 0: ##### PERSONAL NOTE: implement a good exception handling
            raise myInputError(str(self.a)+','+str(self.b)+','+str(self.c)+','+str(self.d),"The matrix must be invertible")
            
        self.oo = extended_complex_plane_CP.numpyExtendedComplexPlane().oo
        self.evaluation = numpy.vectorize(self.EvaluationAtConcretePoint)
    

    

        
    def EvaluationAtConcretePoint(self,z):
        z = extended_complex_plane_CP.numpyExtendedComplexPlane().extendedValue(z)
        if self.theDet == 0:# NECESSARY? PERSONAL NOTE: implement a good exception handling (somewhere, maybe here it wouldn't be necessary if it is well implemented)
            result = "This is not an invertible matrix."# NECESSARY? PERSONAL NOTE: implement a good exception handling (somewhere, maybe here wouldn't be necessary)
        elif z != self.oo and self.c*z + self.d != 0:
            result = (self.a*z + self.b)/(self.c*z + self.d)
        elif z != self.oo and self.c*z + self.d == 0:
            result = self.oo
        elif z == self.oo and self.c != 0:
            result = self.a / self.c
        elif z == self.oo and self.c == 0:
            result = self.oo
        return result
    
    def Mob_trans_iterable(self,z,n): ### PERSONAL NOTE: treat with nth power of matrix instead?
        SingleIteration = self.EvaluationAtConcretePoint
        CurrentPoint = extendedValue(z)
        Orbit = [z]
        for i in range(1,n,1):
            Orbit.append(SingleIteration(CurrentPoint))
            CurrentPoint = SingleIteration(CurrentPoint)
        return Orbit
    
#    def Mob_trans_iterable_oo_removed(self,z,n):
#        Orbit = self.Mob_trans_iterable(z,n)
#        reducedOrbit = [x for x in Orbit if x != 'oo']
#        return reducedOrbit
    
    def fixedPoints(self):
       if self.theDet == 0:# NECESSARY? Perhaps it wouldn't be necessary if a good exception handling were implemented
           fixed_point_set = "This is not an invertible matrix."# NECESSARY?
       elif self.c == 0 and self.a == self.d and self.b == 0:
           fixed_point_set = "This Mobius transformation is the identity. It fixes every point in the extended complex plane."
       elif self.c == 0 and self.a == self.d and self.b != 0:
           fixed_point_set = [self.oo]
       elif self.c == 0 and self.a != self.d:
           z1 =  self.b / (self.d-self.a) 
           fixed_point_set = [z1, self.oo]
       elif self.c != 0:
           #coeffOfQuadraticPol = [self.c,self.d-self.a,-self.b]
           #fixed_point_set = numpy.roots(coeffOfQuadraticPol)
           fixed_point_set = [ (self.a-self.d + numpy.sqrt((self.d-self.a)**2 + 4 * self.b * self.c)) / (2 * self.c) , (self.a-self.d - numpy.sqrt((self.d-self.a)**2 + 4 * self.b * self.c)) / (2 * self.c) ]
           #x = sympy.Symbol('x')
           #quadratic_pol = (self.c*(x**2))+((self.d-self.a)*x)-self.b
           #fixed_point_set = sympy.solve(quadratic_pol,x)
       if len(fixed_point_set) == 1:
           fixed_point_set = [fixed_point_set[0],fixed_point_set[0]]
       return fixed_point_set
   
#    def fixedPoints_oo_removed(self):
#        fixed_point_set = self.fixedPoints()
#        reduced_fixed_point_set = [x for x in fixed_point_set if x != 'oo']
#        return reduced_fixed_point_set
        
    def MobiusTrace(self):
        if self.theDet == 0: # NECESSARY? Perhaps it wouldn't be necessary if a good exception handling were implemented
            traza = "This is not an invertible matrix."
        else:
            traza0 = (self.a+self.d) / (numpy.sqrt(self.theDet)) 
            traza1 = -(traza0)
            traza = [ traza0 , traza1 ]
        return traza
    
    def isParEllHypLox(self):
        if self.theDet == 0:
            TheTypeIs = "This is not an invertible matrix."
        else:
            S = self.fixedPoints()
            T = self.MobiusTrace()
            if self.c == 0 and self.a == self.d and self.b == 0:
                TheTypeIs = "Idendity"
#            elif S[0] == S[1]:
            elif (self.d - self.a)**2 + (4 * self.b * self.c) == 0:
                TheTypeIs = 'PARABOLIC'
            elif numpy.imag(T[0]) ==0 and numpy.absolute(numpy.real(T[0]))<2:
                TheTypeIs = 'ELLIPTIC'
            elif numpy.imag(T[0]) == 0 and numpy.absolute(numpy.real(T[0]))>2:
                TheTypeIs = 'HYPERBOLIC'
            elif numpy.imag(T[0]) !=0:
                TheTypeIs = 'LOXODROMIC'
        return TheTypeIs
    
    def frameOfParabolic(self):
        if self.isParEllHypLox() == 'PARABOLIC':
            fixedPoint = self.fixedPoints()[0]
            if fixedPoint == oo:
                direction = self.EvaluationAtConcretePoint(0)
                angle = 180*numpy.angle(direction)/numpy.pi
                normal = -direction.imag + direction.real*(1j)
                specialPoint = 0
                return [[specialPoint,normal,-normal],angle]
            if fixedPoint != oo:
                testPt1 = fixedPoint + 1
                testPt2 = self.EvaluationAtConcretePoint(testPt1)
                if areCollinear(fixedPoint,testPt1,testPt2) == False:
                    center = e_circumcenter_and_radius(fixedPoint,testPt1,testPt2)[0]
                    complexCenter = center[0] + center[1]*(1j)
                    normal = fixedPoint - complexCenter
                    direction = -normal.imag + normal.real*(1j)
                else:
                    direction = fixedPoint + 1
                    normal = -direction.imag + direction.real*(1j)
                angle = 180*numpy.angle(direction)/numpy.pi
                specialPoint = fixedPoint + direction
                return [[specialPoint,fixedPoint+normal,fixedPoint-normal],angle]
        else:
            raise myInputError(str(self.a)+','+str(self.b)+','+str(self.c)+','+str(self.d),"This function is defined only for parabolic transformations")
                
            
                
                
                
            
    
    def standardForm(self): #### PERSONAL NOTE: This needs a good exception handling
        fixedPointSet = self.fixedPoints()
        alpha1 = fixedPointSet[0]
        alpha2 = fixedPointSet[1]
        if alpha1 == alpha2 and alpha2 != self.oo:
            ConjugatingMatrix = numpy.matrix([[0,1],[1,-alpha2]])
        if alpha1 == alpha2 and alpha2 == self.oo:
            ConjugatingMatrix = numpy.matrix([[1,0],[0,1]])
        if alpha1 != alpha2 and alpha2 != self.oo:
            ConjugatingMatrix = numpy.matrix([[1,-alpha1],[1,-alpha2]])
        if alpha1 != alpha2 and alpha2 == self.oo:
            ConjugatingMatrix = numpy.matrix([[1,-alpha1],[0,1]])
        standard_form = ConjugatingMatrix*self.theMatrix*(ConjugatingMatrix.getI())
        return [standard_form, ConjugatingMatrix]
        
    
#    def SteinerConfiguration(self):
#        if self.theDet == 0 or self.isParEllHypLox()=="Identity":
#            pass
#        elif self.isParEllHypLox() in ['ELLIPTIC','HYPERBOLIC','LOXODROMIC']:
#            fixedPointSet = self.fixedPoints()
#            standard_form = standardForm()
#            conjugating = (standardForm()[1]).getI()
#            n = numpy.random.randint(2,25)
##            intervalForCircle = numpy.linspace(0,2*numpy.pi,n)
##            points = numpy.cos(intervalForCircle) + numpy.sin(intervalForCircle)*(1j)
##            a, b, c, d = conjugating[0,0], conjugating[0,1], conjugating[1,0], conjugating[1,1]
##            pointsForCommon = (a*points + b)/(c*points + d)
#            
            
            
            
  


          
            

        
    


    

    
        