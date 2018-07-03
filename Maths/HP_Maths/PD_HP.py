#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 07:54:19 2018

@author: daniellabardini
"""

import numpy



from exception_handling import myInputError
from Maths.CP_Maths import extended_complex_plane_CP
from Maths.HP_Maths import UHP_PD_Hyper_Isometries_HP
from Maths.HP_Maths.UHP_HP import UHPGeodesicMotion


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
intersection_of_e_circles = extended_complex_plane_CP.numpyExtendedComplexPlane().intersection_of_e_circles
areCollinear = extended_complex_plane_CP.numpyExtendedComplexPlane().areCollinear
myNumpyCosecant = extended_complex_plane_CP.numpyExtendedComplexPlane().myNumpyCosecant
myNumpyCotangent = extended_complex_plane_CP.numpyExtendedComplexPlane().myNumpyCotangent
####



        
class PDBasics:
    
    def __init__(self):
        pass
    
    def isInPD(self,complexP):
        P = extendedValue(complexP)
        if P != oo and numpy.absolute(P) < 1:
            answer = True
        else:
            answer = False
        return answer
    
    def isIdealPoint(self,complexP):
        P = extendedValue(complexP)
        if numpy.absolute(P) == 1:
            answer = True
        else:
            answer = False
        return answer
    
    def isInPDBar(self,complexP):
        if self.isInPD(complexP) == True or self.isIdealPoint(complexP) == True:
            answer = True
        else:
            answer = False
        return answer
        
    def areEuclidCollinearWithCenterInPD(self,hPointP,hPointQ): # NOTE: if one of the points is the center, the answer will be "True"
        P, Q = extendedValue(hPointP), extendedValue(hPointQ)
#        if self.isInPD(P) == True and self.isInPD(Q) == True and P != Q:
        if P!= Q:
            if (P/(P-Q)).imag == 0:
                answer = True
            else:
                answer = False
            return answer
        else:
            raise myInputError(str(P)+','+str(Q),"The points must be distinct")

    def PDNorm(self,point,vector): # PERSONAL NOTE: check if PDNorm yields PDDist indeed (ie, check that no re-scaling is necessary)
        P = numpy.complex(point)
        V = numpy.complex(vector)
        result =  numpy.sqrt( ( 4*(V.real**2+V.imag**2) ) / ((1-P.real**2-P.imag**2)**2) )
        return result

    def PDDist(self,pointP,pointQ): # PERSONAL NOTE: check if PDNorm yields PDDist indeed (ie, check that no re-scaling is necessary)
        P = numpy.complex(pointP)
        Q = numpy.complex(pointQ)
        dist_expression = 2*numpy.arctanh( numpy.absolute( (Q-P) / (1 - numpy.conjugate(P)*Q) ) )
        return dist_expression
    

            
#    def eCenterAndRadiusNonVertGeodesicThroughPAndQ(self,hpointP,hpointQ):
#        P, Q = extendedValue(hpointP), extendedValue(hpointQ)
#        if self.areVertAlignedInUHP(P,Q) == False:
#            normal = (Q-P).imag + (P-Q).real*(1j)
#            midpoint = (P+Q)/2
#            eCenter = midpoint + (-(midpoint.imag)/(normal.imag))*(normal)
#            eRadius = numpy.absolute(P-eCenter)
#            return [eCenter,eRadius]
#        else:
#            raise myInputError(str(P)+','+str(Q),"The points must be distinct, belong to the UHP, and not be vertically alligned")



        
        
            
    def eCenterAndRadiusH_Circle(self,hcenter,hradius): ##### PERSONAL NOTE: WRITE A CODE TO COMPUTE DIRECTLY IN PD, IE, WITHOUT PASSING THROUGH UHP
        UHPcenter = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryPDtoUHP(numpy.complex(hcenter))
        Hradius = numpy.real(hradius)            
        x0 = numpy.real(UHPcenter)
        x1 = numpy.imag(UHPcenter)
        UHPeuclidean_center_x = x0
        UHPeuclidean_center_y = x1*(numpy.cosh(Hradius))
        #euclidean_center = Point(euclidean_center_x, euclidean_center_y)
        UHPeuclidean_radius = x1*(numpy.sinh(Hradius))
        UHPa = UHPeuclidean_center_x + UHPeuclidean_radius + (UHPeuclidean_center_y)*(1j)
        UHPb = UHPeuclidean_center_x + (UHPeuclidean_center_y+ UHPeuclidean_radius)*(1j)
        UHPc = UHPeuclidean_center_x - UHPeuclidean_radius + (UHPeuclidean_center_y)*(1j)
        print(UHPa,UHPb,UHPc)
        PDa = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryUHPtoPD(UHPa)
        PDb = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryUHPtoPD(UHPb)
        PDc = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryUHPtoPD(UHPc)
        print(PDa,PDb,PDc)
        euclideanCenterAndRadius = e_circumcenter_and_radius(PDa,PDb,PDc)
        return [ numpy.complex(euclideanCenterAndRadius[0][0] + euclideanCenterAndRadius[0][1]*(1j)), euclideanCenterAndRadius[1]]

    def PDGeodesicSegment_rcostrsint(self,startpoint,endpoint): # THIS IS THE STATIC SEGMENT, NOT THE ANIMATED SEGMENT PARAMETERIZED BY ARC LENGTH
        w = extendedValue(startpoint)
        z = extendedValue(endpoint)
        if PDBasics().areEuclidCollinearWithCenterInPD(w,z) == True:
            interval = numpy.linspace(0,1)
        else:
            Invz = z / (z.real**2 + z.imag**2)
            Invw = w / (w.real**2 + w.imag**2)
            if numpy.absolute(z) < 1:
                eCenterAndRadius = e_circumcenter_and_radius(w,z,Invz)
                eCenter = eCenterAndRadius[0][0]+eCenterAndRadius[0][1]*(1j)
            elif numpy.absolute(w) < 1:
                eCenterAndRadius = e_circumcenter_and_radius(w,z,Invw)
                eCenter = eCenterAndRadius[0][0]+eCenterAndRadius[0][1]*(1j)
            else:
                eCenter = (w+z)/2
            #eRadius = eCenterAndRadius[1]
            thetaw = numpy.angle(-(w-eCenter))+numpy.pi
            thetaz = numpy.angle(-(z-eCenter))+numpy.pi
            interval = numpy.linspace(0,min(numpy.abs(thetaz-thetaw),2*numpy.pi-numpy.abs(thetaz-thetaw)))
        def parametrized_curve(t):
            if PDBasics().areEuclidCollinearWithCenterInPD(w,z) == True:
                parametrization = (1-t)*w + t*z
            else:
                if 0 <= thetaz-thetaw and thetaz-thetaw <= numpy.pi:
                    parametrization = eCenter + (numpy.absolute(w-eCenter)*(numpy.cos(t+thetaw)+numpy.sin(t+thetaw)*(1j)))
                if thetaz-thetaw > numpy.pi:
                    parametrization = eCenter + (numpy.absolute(w-eCenter)*(numpy.cos(t+thetaz)+numpy.sin(t+thetaz)*(1j)))
                if -numpy.pi <= thetaz - thetaw and thetaz -thetaw <0:
                    parametrization = eCenter + (numpy.absolute(w-eCenter)*(numpy.cos(t+thetaz)+numpy.sin(t+thetaz)*(1j)))
                if thetaz-thetaw < -numpy.pi:
                    parametrization = eCenter + (numpy.absolute(w-eCenter)*(numpy.cos(t+thetaw)+numpy.sin(t+thetaw)*(1j)))
            return parametrization
        return parametrized_curve(interval)
            
            

        
        
        

    



        







    
class PDGeodesicMotion:
    
    def __init__(self):
        pass
    
    def PDGeodesicSegmentParamByArcLength(self,startpoint,endpoint): ## THE OUTPUT IS A FUNCTION, TO BE USED FOR ANIMATION 
        A = extendedValue(startpoint)
        B = extendedValue(endpoint)
        distance = PDBasics().PDDist(A,B)
        def parametrized_curve(s):
            if PDBasics().areEuclidCollinearWithCenterInPD(A,B) == True:
                pass
            else:
                InvA = A / (A.real**2 + A.imag**2)
                eCenterAndRadius = e_circumcenter_and_radius(A,B,InvA)
                eCenter = eCenterAndRadius[0][0]+eCenterAndRadius[0][1]*(1j)
                eRadius = eCenterAndRadius[1]
                P = eCenter + eRadius*(( numpy.tanh(s/2) +A - eCenter - numpy.conjugate(A)*numpy.tanh(s/2)*eCenter ) / ( eRadius*(numpy.conjugate(A)*numpy.tanh(s/2) +1) ))
                Q = eCenter + eRadius*(( -numpy.tanh(s/2) +A - eCenter - numpy.conjugate(A)*(-numpy.tanh(s/2))*eCenter ) / ( eRadius*(numpy.conjugate(A)*(-numpy.tanh(s/2)) +1) ))
                R = eCenter + eRadius*(( ((1j)*numpy.tanh(s/2)) +A - eCenter - numpy.conjugate(A)*((1j)*numpy.tanh(s/2))*eCenter ) / ( eRadius*(numpy.conjugate(A)*((1j)*numpy.tanh(s/2)) +1) ))
                auxCenterAndRadius = e_circumcenter_and_radius(P,Q,R)
                auxCenter = auxCenterAndRadius[0]
                auxRadius = auxCenterAndRadius[1]
                intersectionpoints = intersection_of_e_circles([eCenter.real,eCenter.imag],eRadius,auxCenter,auxRadius)
#                if A.real < B.real:
#                    parametrization = intersectionpoints[0][0] + intersectionpoints[0][1]*(1j)
#                if A.real == B.real and A.imag <= B.imag:
#                    parametrization = intersectionpoints[0][0] + intersectionpoints[0][1]*(1j)
#                if A.real == B.real and A.imag > B.imag:
#                    parametrization = intersectionpoints[1][0] + intersectionpoints[1][1]*(1j)
#                if A.real > B.real:
#                    parametrization = intersectionpoints[1][0] + intersectionpoints[1][1]*(1j)
                intersection0 = intersectionpoints[0][0] + intersectionpoints[0][1]*(1j)
                intersection1 = intersectionpoints[1][0] + intersectionpoints[1][1]*(1j)
                if PDBasics().PDDist(intersection0,B) < PDBasics().PDDist(intersection1,B):
                    parametrization = intersection0
                else:
                    parametrization = intersection1
            return parametrization
        return [parametrized_curve,distance]
    
    #### I AM EDITING HERE HERE HERE HERE
    
    
    
    def UHPGeodesicSegmentConstantRapidity(self,startpoint,initialvelocityvector):
        P = extendedValue(startpoint)
        V = extendedValue(initialvelocityvector)
        rapidity = UHPBasics().UHPNorm(P,V)
        if V.real == 0:
            Q = P + (numpy.sign(V.imag)*(1j))
        else:
            normal = (1j)*V
            eCenter = P + (-(P.imag)/(normal.imag))*(normal)## EUCLIDEAN CENTER OF GEODESIC
            eRadius = numpy.absolute(P-eCenter)
            angle = numpy.angle(P-eCenter)
            Q = eCenter + (eRadius * ( (numpy.sign(V.real)*numpy.cos(angle/2)) + (numpy.sin(angle/2)*(1j)) ))
        def paramCurve(time):
            return self.UHPGeodesicSegmentParamByArcLength(P,Q)[0](rapidity*time)
        return paramCurve
             
            
            
        
        

    
class UHPCircularMotion:
    
    def __init__(self):
        pass
    
    
    def UHPCircSegmentParamByArcLength(self,center,radius,theta1,theta2):
        x0, y0, r = center.real, center.imag, radius
        if y0 <= 0 or y0 <= r:
            raise myInputError(str(y0)+','+str(r),"Center must have positive imaginary part and euclidean radius must be smaller than imaginary part of center")
        else:
            totalarclength = ( 4*r / numpy.sqrt(y0**2-r**2) ) * numpy.arctan( (y0*numpy.tan( theta2/2 ) + r) / numpy.sqrt(y0**2-r**2) ) - ( 4*r / numpy.sqrt(y0**2-r**2) ) * numpy.arctan( (y0*numpy.tan( theta1/2 ) + r) / numpy.sqrt(y0**2-r**2) )
            def parametrization(s):    
                theta = 2*numpy.arctan(((numpy.sqrt(y0**2-r**2)*numpy.tan(( s + ( 4*r / numpy.sqrt(y0**2-r**2) ) * numpy.arctan( (y0*numpy.tan( theta1/2 ) + r) / numpy.sqrt(y0**2-r**2) ) ) * numpy.sqrt(y0**2-r**2) / (4*r) )) - r) / y0)
                return x0+r*numpy.cos(theta) + (y0+r*numpy.sin(theta))*(1j)
            return [parametrization, totalarclength]
        
    
class UHPIsometries:
    
    def __init__(self):
        pass

    
    
    