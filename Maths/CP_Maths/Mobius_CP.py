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
    
    def __init__(self):
                    
        self.oo = extended_complex_plane_CP.numpyExtendedComplexPlane().oo
        self.evaluation = numpy.vectorize(self.EvaluationAtConcretePoint)
    

    

        
    def EvaluationAtConcretePoint(self,complexa,complexb,complexc,complexd):
        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)
        if a*d-b*c == 0:# NECESSARY? PERSONAL NOTE: implement a good exception handling (somewhere, maybe here it wouldn't be necessary if it is well implemented)
            pass#result = "This is not an invertible matrix."# NECESSARY? PERSONAL NOTE: implement a good exception handling (somewhere, maybe here wouldn't be necessary)
        else:
            def MobTrans(complexz):
                z = extendedValue(complexz)#extended_complex_plane_CP.numpyExtendedComplexPlane().extendedValue(complexz)
                if z != oo and c*z + d != 0:
                    result = (a*z + b)/(c*z + d)
                if z != oo and c*z + d == 0:
                    result = oo
                if z == oo and c != 0:
                    result = a / c
                if z == oo and c == 0:
                    result = oo
                return result
            return MobTrans
    
    def MobTrans_nthPowerEvalAtConcretePoint(self,complexa,complexb,complexc,complexd,n):
        a, b, c, d= extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)
        if a*d-b*c == 0:
            pass
        else:
            A = numpy.matrix([[a,b],[c,d]])**n
            alpha, beta, gamma, delta = A[0,0], A[0,1], A[1,0], A[1,1]
            def MobTrans(complexz):
                z = extendedValue(complexz)
                if z != oo and gamma*z + delta != 0:
                    result = (alpha*z + beta)/(gamma*z + delta)
                if z != oo and gamma*z + delta == 0:
                    result = oo
                if z == oo and gamma != 0:
                    result = alpha / gamma
                if z == oo and gamma == 0:
                    result = oo
                return result
            return MobTrans
            
            
            
    
    def MobTransOrbit(self,complexa,complexb,complexc,complexd,m):
        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)
        if a*d-b*c == 0:
            pass
        else:
            def theOrbit(complexz):
                OrbitAllPts = {numpy.sign(m)*n:self.MobTrans_nthPowerEvalAtConcretePoint(complexa,complexb,complexc,complexd,numpy.sign(m)*n)(complexz) for n in range(0,abs(m)+1,1)}
                OrbitFinitePts = {numpy.sign(m)*n:self.MobTrans_nthPowerEvalAtConcretePoint(complexa,complexb,complexc,complexd,numpy.sign(m)*n)(complexz) for n in range(0,abs(m)+1,1) if self.MobTrans_nthPowerEvalAtConcretePoint(complexa,complexb,complexc,complexd,numpy.sign(m)*n)(complexz) != oo}
                OrbitInfinitePts = {numpy.sign(m)*n:self.MobTrans_nthPowerEvalAtConcretePoint(complexa,complexb,complexc,complexd,numpy.sign(m)*n)(complexz) for n in range(0,abs(m)+1,1) if self.MobTrans_nthPowerEvalAtConcretePoint(complexa,complexb,complexc,complexd,numpy.sign(m)*n)(complexz) == oo}
                return [OrbitAllPts, OrbitFinitePts, OrbitInfinitePts]
            return theOrbit


#    def Mob_trans_iterable_oo_removed(self,z,n):
#        Orbit = self.Mob_trans_iterable(z,n)
#        reducedOrbit = [x for x in Orbit if x != 'oo']
#        return reducedOrbit
    
    def fixedPoints(self,complexa,complexb,complexc,complexd):
        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)
        if a*d-b*c == 0:
            pass
        else:
            if c == 0 and a == d and b == 0:
                pass
            else:
                if c == 0 and a == d and b != 0:
                    fixed_point_set = [oo]
                if c == 0 and a != d:
                    z1 =  b / (d-a) 
                    fixed_point_set = [z1, oo]
                if c != 0:
               #coeffOfQuadraticPol = [self.c,self.d-self.a,-self.b]
               #fixed_point_set = numpy.roots(coeffOfQuadraticPol)
                   fixed_point_set = [ (a-d + numpy.sqrt((d-a)**2 + 4 * b * c)) / (2 * c) , (a-d - numpy.sqrt((d-a)**2 + 4 * b * c)) / (2 * c) ]
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

    def standardForm(self,complexa,complexb,complexc,complexd): #### PERSONAL NOTE: This needs a good exception handling
        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)
        if a*d-b*c == 0:
            pass
        else:
            if c == 0 and a == d and b == 0:
                pass
            else:
                A = numpy.matrix([[a,b],[c,d]])
                fixedPointSet = self.fixedPoints(a,b,c,d)
                alpha1 = fixedPointSet[0]
                alpha2 = fixedPointSet[1]
                if alpha1 == alpha2 and alpha2 != oo:
                    ConjugatingMatrix = numpy.matrix([[0,1],[1,-alpha2]])
                if alpha1 == alpha2 and alpha2 == oo:
                    ConjugatingMatrix = numpy.matrix([[1,0],[0,1]])
                if alpha1 != alpha2 and alpha2 != oo:
                    ConjugatingMatrix = numpy.matrix([[1,-alpha1],[1,-alpha2]])
                if alpha1 != alpha2 and alpha2 == oo:
                    ConjugatingMatrix = numpy.matrix([[1,-alpha1],[0,1]])
                standard_form = ConjugatingMatrix*A*(ConjugatingMatrix**(-1))
                return [standard_form, ConjugatingMatrix]
        
        
    def MobiusTrace(self,complexa,complexb,complexc,complexd):
        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)        
        if a*d-b*c == 0:
            pass
        else:
            traza0 = (a+d) / (numpy.sqrt(a*d-b*c)) 
            traza1 = -(traza0)
            traza = [ traza0 , traza1 ]
        return traza
    
#    def isParEllHypLox(self,complexa,complexb,complexc,complexd):
#        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)        
#        if a*d-b*c == 0:
#            pass
#        else:
#            #S = self.fixedPoints(a,b,c,d)
#            #T = self.MobiusTrace(a,b,c,d)
#            if c == 0 and a == d and b == 0:
#                TheTypeIs = "Idendity"
##            elif S[0] == S[1]:
#            elif (d - a)**2 + (4 * b * c) == 0:
#                TheTypeIs = 'Parabolic'
#            elif ((a+d)**2 / (a*d-b*c) ).imag == 0 and 0 <= ((a+d)**2 / (a*d-b*c)).real < 4:
#                TheTypeIs = 'Elliptic'
#            elif ((a+d)**2 / (a*d-b*c)).imag == 0 and ((a+d)**2 / (a*d-b*c)).real > 4:
#                TheTypeIs = 'Hyperbolic'
#            elif ((a+d)**2 / (a*d-b*c) ).imag != 0 or ((a+d)**2 / (a*d-b*c)).real < 0:
#                TheTypeIs = 'Loxodromic'
#        return TheTypeIs

    def isParEllHypLox(self,complexa,complexb,complexc,complexd):
        a, b, c, d = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd)        
        if a*d-b*c == 0:
            pass
        else:
            #S = self.fixedPoints(a,b,c,d)
            #T = self.MobiusTrace(a,b,c,d)
            if c == 0 and a == d and b == 0:
                TheTypeIs = ["Idendity",1]
#            elif S[0] == S[1]:
            else:
                normalForm = self.standardForm(a,b,c,d)[0]
                alpha,beta,gamma,delta = normalForm[0,0],normalForm[0,1],normalForm[1,0],normalForm[1,1]
                mu = self.EvaluationAtConcretePoint(alpha,beta,gamma,delta)(1)
                if (d - a)**2 + (4 * b * c) == 0:
                    TheTypeIs = ['Parabolic',self.EvaluationAtConcretePoint(alpha,beta,gamma,delta)(0)]
                elif numpy.absolute(mu) == 1:
                    TheTypeIs = ['Elliptic', mu]
                elif mu.imag == 0 and mu.real > 0:
                    TheTypeIs = ['Hyperbolic',mu]
                else:
                    TheTypeIs = ['Loxodromic',mu]
            return TheTypeIs
        
    def invariantCurveThroughPt(self,complexa,complexb,complexc,complexd,z):### PERSONAL NOTE: refine this so it handles the case when the orbit of z is contained in an euclidean line
        a, b, c, d, z = extendedValue(complexa), extendedValue(complexb), extendedValue(complexc), extendedValue(complexd), extendedValue(z)
        if a*d-b*c == 0:
            pass
        elif c == 0 and a == d and b == 0:
            pass
        else:
            z1 = self.EvaluationAtConcretePoint(a,b,c,d)(z)
            z2 = self.EvaluationAtConcretePoint(a,b,c,d)(z1)
            if areAllDistinctArgs(z,z1,z2) == False:
                coordList = []
            elif areCollinear(z,z1,z2) == True:
                coordList = []
            else:
                Type = self.isParEllHypLox(a,b,c,d)
                coordList = []
                if Type[0] == 'Parabolic' or Type[0] == 'Elliptic' or Type[0] == 'Hyperbolic':
                    center = e_circumcenter_and_radius(z,z1,z2)[0]
                    radius = e_circumcenter_and_radius(z,z1,z2)[1]
                    t = numpy.linspace(0,2*numpy.pi,1000)
                    curve = (center[0]+radius*numpy.cos(t))+(center[1]+radius*numpy.sin(t))*(1j)
                    x_coord = curve.real
                    y_coord = curve.imag
                    coordList.append([x_coord,y_coord])
                if Type[0] == 'Loxodromic':
                    P = self.fixedPoints(a,b,c,d)[0]
                    Q = self.fixedPoints(a,b,c,d)[1]
                    mu = Type[1]
                    C = self.standardForm(a,b,c,d)[1]
                    w = self.EvaluationAtConcretePoint(C[0,0],C[0,1],C[1,0],C[1,1])(z)
                    t = numpy.linspace(0,2*numpy.pi,10000)
                    if isooInArgs(P,Q) == True:
                        curve1 = P+(((mu)**(t**2))*w)
                        curve2 = P+(((mu**(-1))**(t**2))*w)
                        x_coord1 = curve1.real
                        y_coord1 = curve1.imag
                        x_coord2 = curve2.real
                        y_coord2 = curve2.imag
                        coordList.append([x_coord1,y_coord1])  
                        coordList.append([x_coord2,y_coord2])  
                    else:
                        complexW = ((mu)**(t**2))*w
                        complexZ = (((mu)**(-1))**(t**2))*w
                        D = C**(-1)
                        x_coord = ((D[0,0]*complexW + D[0,1])/(D[1,0]*complexW + D[1,1])).real
                        y_coord = ((D[0,0]*complexW + D[0,1])/(D[1,0]*complexW + D[1,1])).imag
    #                            x_coord = ((Q*complexW + P)/(complexW + 1)).real
    #                            y_coord = ((Q*complexW + P)/(complexW + 1)).imag
                        coordList.append([x_coord,y_coord])
                        x_coord2 = ((D[0,0]*complexZ + D[0,1])/(D[1,0]*complexZ + D[1,1])).real
                        y_coord2 = ((D[0,0]*complexZ + D[0,1])/(D[1,0]*complexZ + D[1,1])).imag
    #                            x_coord2 = ((P*complexW + Q)/(complexW + 1)).real
    #                            y_coord2 = ((P*complexW + Q)/(complexW + 1)).imag
                        coordList.append([x_coord2,y_coord2])
            return coordList

    
#    def frameOfParabolic(self):
#        if self.isParEllHypLox() == 'PARABOLIC':
#            fixedPoint = self.fixedPoints()[0]
#            if fixedPoint == oo:
#                direction = self.EvaluationAtConcretePoint(0)
#                angle = 180*numpy.angle(direction)/numpy.pi
#                normal = -direction.imag + direction.real*(1j)
#                specialPoint = 0
#                return [[specialPoint,normal,-normal],angle]
#            if fixedPoint != oo:
#                testPt1 = fixedPoint + 1
#                testPt2 = self.EvaluationAtConcretePoint(testPt1)
#                if areCollinear(fixedPoint,testPt1,testPt2) == False:
#                    center = e_circumcenter_and_radius(fixedPoint,testPt1,testPt2)[0]
#                    complexCenter = center[0] + center[1]*(1j)
#                    normal = fixedPoint - complexCenter
#                    direction = -normal.imag + normal.real*(1j)
#                else:
#                    direction = fixedPoint + 1
#                    normal = -direction.imag + direction.real*(1j)
#                angle = 180*numpy.angle(direction)/numpy.pi
#                specialPoint = fixedPoint + direction
#                return [[specialPoint,fixedPoint+normal,fixedPoint-normal],angle]
#        else:
#            raise myInputError(str(self.a)+','+str(self.b)+','+str(self.c)+','+str(self.d),"This function is defined only for parabolic transformations")
#                
#            
#                
                
                
            
    

    
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
            
            
            
class MobiusTransitivity:
    
    def __init__(self):
        pass
    
    def MobiusTransz1z2z3To0oo1(self,Z1,Z2,Z3):
        z1,z2,z3 = extendedValue(Z1), extendedValue(Z2), extendedValue(Z3)
        if areAllDistinctArgs(z1,z2,z3) != True:
            pass
        else:
            def MobiusTrans(z):
                if z1 == oo:
                    if z == oo:
                        result = 0
                    elif z == z2:
                        result = 0
                    else:
                        result = (z3-z2)/(z-z2)
                elif z2 == oo:
                    if z == oo:
                        result = oo
                    else:
                        result = (z - z1) / (z3 - z1)
                elif z3 == oo:
                    if z == oo:
                        result = 1
                    elif z == z2:
                        result = oo
                    else:
                        result = (z - z1) / (z-z2)
                else: 
                   result = ((z3-z2)*(z - z1))/((z3-z1)*(z - z2))
                return result
            return MobiusTrans
        
    def MobiusMatrixz1z2z3To0oo1(self,Z1,Z2,Z3):
        z1,z2,z3 = extendedValue(Z1), extendedValue(Z2), extendedValue(Z3)
        if areAllDistinctArgs(z1,z2,z3) != True:
            pass
        else:
            if z1 == oo:
                A = numpy.matrix([[0,z3-z2],[1,-z2]])
            elif z2 == oo:
                A = numpy.matrix([[1,-z1],[0,z3-z1]])
            elif z3 == oo:
                A = numpy.matrix([[1,-z1],[1-z2]])
            else:
                A = numpy.matrix([[z3-z2,(z2-z3)*z1],[z3-z1,(z1-z3)*z2]])
            return A
        
    def MobiusMatrix0oo1Toz1z2z3(self,Z1,Z2,Z3):
        z1,z2,z3 = extendedValue(Z1), extendedValue(Z2), extendedValue(Z3)
        if areAllDistinctArgs(z1,z2,z3) != True:
            pass
        else:
            return self.MobiusMatrixz1z2z3To0oo1(z1,z2,z3)**(-1)
        
    def MobiusMatrixz1z2z3Tow1w2w3(self,Z1,Z2,Z3,W1,W2,W3):
        z1,z2,z3,w1,w2,w3 = extendedValue(Z1), extendedValue(Z2), extendedValue(Z3), extendedValue(W1), extendedValue(W2), extendedValue(W3)
        if areAllDistinctArgs(z1,z2,z3) != True or areAllDistinctArgs(w1,w2,w3) != True:
            pass
        else:
            A = self.MobiusMatrixz1z2z3To0oo1(Z1,Z2,Z3)
            B = self.MobiusMatrix0oo1Toz1z2z3(W1,W2,W3)
            return B*A
        
    def MobiusTransz1z2z3Tow1w2w3(self,Z1,Z2,Z3,W1,W2,W3):
        z1,z2,z3,w1,w2,w3 = extendedValue(Z1), extendedValue(Z2), extendedValue(Z3), extendedValue(W1), extendedValue(W2), extendedValue(W3)
        if areAllDistinctArgs(z1,z2,z3) != True or areAllDistinctArgs(w1,w2,w3) != True:
            pass
        else:
            theMatrix = self.MobiusMatrixz1z2z3Tow1w2w3(Z1,Z2,Z3,W1,W2,W3)
            a,b,c,d = theMatrix[0,0], theMatrix[0,1], theMatrix[1,0], theMatrix[1,1]
            def Transformation(z):
                return MobiusAssocToMatrix().EvaluationAtConcretePoint(a,b,c,d)(z)
            return Transformation
                
    def crossRatio(self,Z1,Z2,Z3,Z4):
        z1,z2,z3,z4 = extendedValue(Z1), extendedValue(Z2), extendedValue(Z3), extendedValue(Z4)
        if areAllDistinctArgs(z1,z2,z3) != True:
            pass
        else:
            MobTrans = self.MobiusTransz1z2z3To0oo1(z1,z3,z2)
            return MobTrans(z4)
        
        
class MobiusFromParameters:
    
    def __init__(self):
        pass

    
    def MobMatrixFromParams(self,listOfFixedPoints,complexalpha):
        if len(listOfFixedPoints) == 0 or len(listOfFixedPoints)>2 or complexalpha == 0 or complexalpha==1:
            pass
        else:
            if len(listOfFixedPoints) == 1 or listOfFixedPoints[0] == listOfFixedPoints[1]:
                z0 = listOfFixedPoints[0]
                can = numpy.matrix([[1,complexalpha],[0,1]])
                if z0 != oo:
                    C = numpy.matrix([[0,1],[1,-z0]])
                else:
                    C = numpy.matrix([1,0],[0,1])
            else:
                z0, z1 = listOfFixedPoints[0], listOfFixedPoints[1]
                can = numpy.matrix([[complexalpha,0],[0,1]])
                if numpy.absolute(complexalpha) == 1: # turning around z0 is counterclockwise
                    if z1 != oo:
                        if z0!=oo:
                            C = numpy.matrix([[1,-z0],[1,-z1]])
                        else:
                            C = numpy.matrix([[0,1],[1,-z1]])
                    else:
                        C = numpy.matrix([[1,-z0],[0,1]])
                else: # z0 is the repulsor, z1 is the attractor, IF numpy.absolute(complexalpha) > 1
                    if z1 != oo:
                        if z0!=oo:
                            C = numpy.matrix([[1,-z0],[1,-z1]])
                        else:
                            C = numpy.matrix([[0,1],[1,-z1]])
                    else: 
                        C = numpy.matrix([[1,-z0],[0,1]])
            result = (C**(-1))*can*C
        return result
                        
                    
                

                

            
                

          
            

        
    


    

    
        