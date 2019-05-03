#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 07:54:19 2018

@author: daniellabardini
"""

import numpy



from exception_handling import myInputError
from Maths.CP_Maths import extended_complex_plane_CP
from Maths.CP_Maths import Mobius_CP
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
myarg0To2Pi = extended_complex_plane_CP.numpyExtendedComplexPlane().myarg0To2Pi
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
        if P != Q:
            if (P/(P-Q)).imag == 0:
                answer = True
            else:
                answer = False
            return answer
        else:
            pass

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
    

            
    def eCenterAndRadiusNonStraightGeodesicThroughPAndQ(self,hpointP,hpointQ):
        P, Q = extendedValue(hpointP), extendedValue(hpointQ)
        if self.areEuclidCollinearWithCenterInPD(P,Q) == True:
            pass
        else:
            if numpy.absolute(P) < 1:
                R = P/(P.real**2+P.imag**2)
                eCenter = e_circumcenter_and_radius(P,Q,R)[0][0]+e_circumcenter_and_radius(P,Q,R)[0][1]*(1j)
            if numpy.absolute(P) == 1 and numpy.absolute(Q) < 1 :
                R = Q/(Q.real**2+Q.imag**2)
                eCenter = e_circumcenter_and_radius(P,Q,R)[0][0]+e_circumcenter_and_radius(P,Q,R)[0][1]*(1j)
            if numpy.absolute(P) == 1 and numpy.absolute(Q) == 1 :
                eCenter = Q+ ((P-Q)/(P+Q))*Q
            eRadius = numpy.absolute(P-eCenter)
            return [eCenter,eRadius]
         
            



        
        
            
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
        PDa = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryUHPtoPD(UHPa)
        PDb = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryUHPtoPD(UHPb)
        PDc = UHP_PD_Hyper_Isometries_HP.CanonicalIsometriesBetweenModels().IsometryUHPtoPD(UHPc)
        euclideanCenterAndRadius = e_circumcenter_and_radius(PDa,PDb,PDc)
        return [ numpy.complex(euclideanCenterAndRadius[0][0] + euclideanCenterAndRadius[0][1]*(1j)), euclideanCenterAndRadius[1]]

    def PDGeodesicSegment_rcostrsint(self,startpoint,endpoint): # THIS IS THE STATIC SEGMENT, NOT THE ANIMATED SEGMENT PARAMETERIZED BY ARC LENGTH
        w = extendedValue(startpoint)
        z = extendedValue(endpoint)
        Invz = z / (z.real**2 + z.imag**2)
        Invw = w / (w.real**2 + w.imag**2)
        if PDBasics().areEuclidCollinearWithCenterInPD(w,z) == False:
            if Invz != z:#numpy.absolute(z) < 1:
                eCenterAndRadius = e_circumcenter_and_radius(w,z,Invz)
                eCenter = eCenterAndRadius[0][0]+eCenterAndRadius[0][1]*(1j)
            elif Invw != w:#numpy.absolute(w) < 1:
                eCenterAndRadius = e_circumcenter_and_radius(w,z,Invw)
                eCenter = eCenterAndRadius[0][0]+eCenterAndRadius[0][1]*(1j)
            else:
                eCenter = w*( 1 +  (z-w)/(z+w))
            thetaw = myarg0To2Pi(w-eCenter)
            thetaz = myarg0To2Pi(z-eCenter)
            if numpy.abs(thetaz-thetaw) <= numpy.pi:
                interval = numpy.linspace(0,thetaz-thetaw)
            elif thetaz-thetaw > numpy.pi:
                interval = numpy.linspace(0,-((2*numpy.pi)-(thetaz-thetaw)))
            elif thetaz-thetaw < numpy.pi:
                interval = numpy.linspace(0,((2*numpy.pi)-numpy.abs(thetaz-thetaw)))
            def parametrized_curve(t):
                parametrization = eCenter + (w-eCenter)*(numpy.cos(t)+numpy.sin(t)*(1j))
                return parametrization
#                matrix = numpy.matrix([[((1j)*w).real,-(1j)*z.real],[((1j)*w).imag,-(1j)*z.imag]])
#                vector = numpy.matrix([[z.real-w.real],[z.imag-w.imag]])
#                scalar = ((matrix**(-1))*vector)[0,0]
#                eCenter = w + scalar*(1j)*w ## EUCLIDEAN CENTER OF GEODESIC
#            #eRadius = eCenterAndRadius[1]
        else:
            interval = numpy.linspace(0,1)
            def parametrized_curve(t):
                parametrization = (1-t)*w + t*z
                return parametrization
        return parametrized_curve(interval)
            
            
####################
#### FOR POLYGONS IN PD

###### TANGENT UNIT VECTOR OF A GIVEN DIRECTED GEODESIC AT A GIVEN POINT

    def tangent_unit(self,point1,point2): ## point1 and point2 must belong to the interior of PD
        P1, P2 = extendedValue(point1), extendedValue(point2)
        if P1 == P2:
            pass
        else:
            if self.areEuclidCollinearWithCenterInPD(P1,P2) == True:
                unit_vector = (P2-P1)/self.PDNorm(P1,(P2-P1))
            if self.areEuclidCollinearWithCenterInPD(P1,P2) == False:
                Q = P1/((P1.real)**2+(P1.imag)**2)
                center = e_circumcenter_and_radius(P1,P2,Q)[0][0]+e_circumcenter_and_radius(P1,P2,Q)[0][1]*(1j)
                connecting_vector = P1-center
                orthogonal_vector = connecting_vector*(1j)
                sign = numpy.sign(numpy.angle(((P2-center)/(P1-center))))
                unit_vector =  sign*orthogonal_vector/self.PDNorm(P1,orthogonal_vector)
            return unit_vector
        
    def tangent_unit_Hbar(self,point1,point2):
        P1, P2 = extendedValue(point1), extendedValue(point2)
        if P1 == P2:
            pass
        else:
            if numpy.absolute(P1) < 1:
                result = self.tangent_unit(P1,P2)
            if numpy.absolute(P1) == 1:
                result = -P1
            return result
        
        
#### CYCLIC ORDERING OF A FAMILY OF POINTS AROUND A FIXED BASEPOINT

    def cyclic_order_counter_clockwise(self,basepoint,vertices):
        point1 = vertices[0]
        reference_vector = self.tangent_unit_Hbar(basepoint,point1)
        remaining_vertices = [k for k in vertices]
        list_of_arguments = []
        for point in remaining_vertices:
            vector = self.tangent_unit_Hbar(basepoint,point)
            difference_of_arguments_in_radians = myarg0To2Pi(vector/reference_vector)#numpy.angle(vector/reference_vector)
            list_of_arguments.append(difference_of_arguments_in_radians)
        ordered_list_of_arguments = sorted(list_of_arguments)
        for t in list_of_arguments:
            if ordered_list_of_arguments.count(t)>1:
                ordered_list_of_arguments.remove(t)
        cyclic_order = []
        for t in ordered_list_of_arguments:
            for point in remaining_vertices:
                vector = self.tangent_unit_Hbar(basepoint,point)
                difference_of_arguments_in_radians = myarg0To2Pi(vector/reference_vector)#numpy.angle(vector/reference_vector)
                if t == difference_of_arguments_in_radians:
                    cyclic_order.append(point)        
        return cyclic_order
            

    def is_point_in_h_convex_hull(self,point,points):
        remaining_vertices = [k for k in points]
        for k in points:
            if remaining_vertices.count(k)>1:
                remaining_vertices.remove(k)
        cyclic_order = self.cyclic_order_counter_clockwise(point,remaining_vertices)
        first_element = cyclic_order[0]
        cyclic_order.append(first_element)
        arguments = []
        for i in range(0,len(cyclic_order)-1,1):
            arguments.append( myarg0To2Pi( self.tangent_unit_Hbar(point,cyclic_order[i+1])/ self.tangent_unit_Hbar(point,cyclic_order[i]) ))
        if max(arguments) > numpy.pi or numpy.absolute(point)==1:
            result = False
        else:
            result = True
        return result




    def is_point_in_interior_h_convex_hull(self,point,points):
        remaining_vertices = [k for k in points]
        for k in points:
            if remaining_vertices.count(k)>1:
                remaining_vertices.remove(k)
        if point in remaining_vertices:
            remaining_vertices.remove(point)
        cyclic_order = self.cyclic_order_counter_clockwise(point,remaining_vertices)
        first_element = cyclic_order[0]
        cyclic_order.append(first_element)
    #    print(cyclic_order)
        arguments = []
        for i in range(0,len(cyclic_order)-1,1):
            arguments.append( myarg0To2Pi (self.tangent_unit_Hbar(point,cyclic_order[i+1])/ self.tangent_unit_Hbar(point,cyclic_order[i]) ))
    #    print(arguments)
        if max(arguments) >= numpy.pi or numpy.absolute(point)==1:
                       result = False
        else:
            result = True
        return result


        

            
    
        
                
    def find_verts_of_h_convex_hull(self,points):
        vert = [k for k in points]
        for k in points:
            if vert.count(k)>1:
                vert.remove(k)
        vertices = [k for k in vert]
        #print(vertices)
        for j in vert:
            index = vertices.index(j)
            vertices.remove(j)
            #print(vertices)
            if self.is_point_in_h_convex_hull(j,vertices) == False:
                vertices.insert(index,j)
            #print(vertices)
        result = vertices
        return result
                
                
            
    def somePointOnGeodesicSegment(self,point1,point2): ## THE OUTPUT POINT IS NOTHING SPECIAL, IT WILL BE USED ONLY TO PROVIDE A BASEPOINT AROUND WHICH TO ORDER CYCLICALLY
        A = extendedValue(point1)
        B = extendedValue(point2)
        if self.areEuclidCollinearWithCenterInPD(A,B) == False:
            X = self.eCenterAndRadiusNonStraightGeodesicThroughPAndQ(A,B)
            t = numpy.angle((B - (X[0]))/(A - (X[0]))) / 2
            point = X[0] + (A - (X[0]))*( numpy.cos(t) + numpy.sin(t)*(1j) )
        else:
            point = (A+B)/2
        return point
        
    

    def verts_h_polygon_counter_clockwise(self,points): ## requires at least two different points in points
        points_no_repetition = [j for j in points]
        for j in points:
            if points_no_repetition.count(j)>1:
                points_no_repetition.remove(j)
    ##    for k in points_no_repetition:
    ##        euclidean_circle(k,5)
        vert = self.find_verts_of_h_convex_hull(points_no_repetition)
        ideal_vert=[]
        for ell in vert:
            if numpy.absolute(ell) == 1:
                ideal_vert.append(ell)
        finite_vert=[]
        for ell in vert:
            if numpy.absolute(ell) < 1:
                finite_vert.append(ell)
        basepoint = self.somePointOnGeodesicSegment(vert[0],vert[1])
        if finite_vert != []:
            finite_vert.extend(ideal_vert)
            vertices = self.cyclic_order_counter_clockwise(basepoint,finite_vert)
            #vertices.insert(0,basepoint)
            p0=vertices[0]
            vertices.append(p0)
        if finite_vert == []:
            vertices = self.cyclic_order_counter_clockwise((1j),vert)
            p0=vertices[0]
            vertices.append(p0)
        return vertices

        
                    

        
        
        

    



        







    
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
    
    

             
            
    def PDGeodesicSegmentConstantRapidity(self,startpoint,initialvelocityvector):
        P = extendedValue(startpoint)
        V = extendedValue(initialvelocityvector)
        rapidity = PDBasics().PDNorm(P,V)
        if P == 0:
            Q = V/(2*numpy.absolute(V))
        elif (V/P).imag == 0:
            if (V/P).real >= 0:
                Q = (P+(P/numpy.absolute(P)))/2
            if (V/P).real < 0:
                Q = -P
        else:
            A = P + V
            B = P - V
            invA = A / (A.real**2 + A.imag**2)
            invB = B / (B.real**2 + B.imag**2)
            invP = P / (P.real**2 + P.imag**2)
            AuxeCenterAndRadius = e_circumcenter_and_radius(invA,invB,invP)
            AuxeCenter = AuxeCenterAndRadius[0][0]+AuxeCenterAndRadius[0][1]*(1j)
            W = invP-AuxeCenter
#            P + t(1j)V = invP + s W
            matrix = numpy.matrix([[((1j)*V).real,-W.real],[((1j)*V).imag,-W.imag]])
            vector = numpy.matrix([[invP.real-P.real],[invP.imag-P.imag]])
            scalar = ((matrix**(-1))*vector)[0,0]
            eCenter = P + scalar*(1j)*V ## EUCLIDEAN CENTER OF GEODESIC
            eRadius = numpy.absolute(P-eCenter)
            angle = numpy.angle(P-eCenter)
            intersectionPts = intersection_of_e_circles([eCenter.real,eCenter.imag],eRadius,[0,0],1)
            intPt1 = intersectionPts[0][0]+intersectionPts[0][1]*(1j)
            intPt2 = intersectionPts[1][0]+intersectionPts[1][1]*(1j)
            auxArg1 = myarg0To2Pi(intPt1-eCenter)
            auxArg2 = myarg0To2Pi(intPt2-eCenter)
            if auxArg2-auxArg1 <= -numpy.pi:
                auxAngle1 = numpy.angle(intPt1-eCenter)
                auxAngle2 = numpy.angle(intPt2-eCenter)                
            if -numpy.pi < auxArg2-auxArg1 < 0:
                auxAngle1 = numpy.angle(intPt2-eCenter)
                auxAngle2 = numpy.angle(intPt1-eCenter)                
            if 0 < auxArg2-auxArg1 < numpy.pi:
                auxAngle1 = numpy.angle(intPt1-eCenter)
                auxAngle2 = numpy.angle(intPt2-eCenter)
            if numpy.pi <= auxArg2-auxArg1:
                auxAngle1 = numpy.angle(intPt2-eCenter)
                auxAngle2 = numpy.angle(intPt1-eCenter)
            if (V/((P-eCenter)*(1j))).real >= 0: #V points counterclockwise on the geodesic
                theta = (auxAngle2+angle)/2
                Q = eCenter + ( eRadius * (numpy.cos(theta)+numpy.sin(theta)*(1j)) )
            if (V/((P-eCenter)*(1j))).real < 0: #V points clockwise on the geodesic
                theta = (angle + auxAngle1)/2
                Q = eCenter + ( eRadius * (numpy.cos(theta)+numpy.sin(theta)*(1j)) )
        def paramCurve(time):
            return self.PDGeodesicSegmentParamByArcLength(P,Q)[0](rapidity*time)
        return paramCurve            
        
        

    
class PDCircularMotion:
    
    def __init__(self):
        pass
    
    
  
    
class PDIsometries:
    
    def __init__(self):
        pass

    
class PDFuchsianRepresentative: ##NOTE: SO FAR, THIS CLASS CONSTRUCTS
# A VERY SPECIFIC FUCHSIAN GROUP IN A VERY SPECIFIC SETTING:
# POSITIVE NUMBER OF PUNCTURES, NO ORBIFOLD POINTS, VERY SPECIFIC SET OF SIDE PAIRING TRANSFORMATIONS
    
    def __init__(self):
        pass

    def PDSpecificCombinatorialSidePairing(self,genus,numberOfPunctures):
        g, p = genus, numberOfPunctures
        if g == 0 and p < 3:
            pass
        elif g == 1 and p == 0:
            pass
        else:
            def f(k):
                if k in range(4*(g-1)):
                    if k % 4 == 0 or k % 4 == 1 :
                        result = k + 2
                    if k % 4 == 2 or k % 4 == 3 :
                        result = k-2
                if k == 4*(g-1):
                    result = 4*(g-1) + 1 + p
                if k == 4*(g-1) + 1 + p:
                    result = 4*(g-1)
                if k in range(4*(g-1)+1,4*(g-1)+1+p):
                    result = 4*(g-1)+p+2 + 4*(g-1)+p-k
                if k in range(4*(g-1)+p+2,4*g+2*(p-1)):
                    result = 4*(g-1)+1 + 4*g+2*(p-1)-1-k
                return result
            return f
    
    def PDSidesOfSpecificIdealPolygon(self,genus,numberOfPunctures):
        # NOTE: TRY TO IMPROVE THE WAY THE SIDES ARE COLORED
        g, p = genus, numberOfPunctures
        if g == 0 and p < 3:
            pass
        elif g == 1 and p == 0:
            pass
        else:
            NumOfSides = (4*g) + (2*(p-1))
            curves = {}
            midpoints = {}
            for k in range(NumOfSides):
                w = numpy.cos(2*k*numpy.pi/NumOfSides)+numpy.sin(2*k*numpy.pi/NumOfSides)*(1j)
                z = numpy.cos(2*(k+1)*numpy.pi/NumOfSides)+numpy.sin(2*(k+1)*numpy.pi/NumOfSides)*(1j)
                eCenter = w*( 1 +  (z-w)/(z+w))
                thetaw = myarg0To2Pi(w-eCenter)#numpy.angle(-(w-eCenter))+numpy.pi
                thetaz = myarg0To2Pi(z-eCenter)#numpy.angle(-(z-eCenter))+numpy.pi
                interval = numpy.linspace(0,min(numpy.abs(thetaz-thetaw),(2*numpy.pi)-numpy.abs(thetaz-thetaw)))
                def parametrized_curve(t):
                    parametrization = eCenter + (z-eCenter)*(numpy.cos(t)+numpy.sin(t)*(1j))
                    return parametrization
                curves[k] = parametrized_curve(interval)
                midpoints[k] = parametrized_curve((min(numpy.abs(thetaz-thetaw),(2*numpy.pi)-numpy.abs(thetaz-thetaw)))/2)
            basicColors = ["m", "b", "r", "c", "g", "y", "w", "k"]
            curvesColors = {4*(g-1):basicColors[2*(g-1)%len(basicColors)],4*(g-1)+p+1:basicColors[2*(g-1)%len(basicColors)]}
#            l = len(basicColors)
#            for k in range(4*(g-1)-2):
#                if k % l == 0 or k % l == 4  :
#                    curvesColors[k] = basicColors[int(k/2)%len(basicColors)]
#                    curvesColors[k+2] = basicColors[int(k/2)%len(basicColors)]  
#                if  k % l == 3 or k % l == 7 :
#                    curvesColors[k] = basicColors[int((k-1)/2)%len(basicColors)]
#                    curvesColors[k+2] = basicColors[int((k-1)/2)%len(basicColors)]
            f = self.PDSpecificCombinatorialSidePairing(g,p)
            for k in range(4*(g-1)+1+p):#range(4*(g-1)+1,4*(g-1)+1+p):
                curvesColors[k] = basicColors[k%len(basicColors)]
                curvesColors[f(k)] = basicColors[k%len(basicColors)]
            result = {"curves":curves,"midpoints":midpoints,"curvesColors":curvesColors}
            #print(curves)
            return result

    def PDSidePairingsOfSpecificIdealPolygon(self,genus,numberOfPunctures):
        g, p = genus, numberOfPunctures
        if g == 0 and p < 3:
            pass
        elif g == 1 and p == 0:
            pass
        else:
            f = self.PDSpecificCombinatorialSidePairing(g,p)
            midpoints = self.PDSidesOfSpecificIdealPolygon(g,p)["midpoints"]
            NumOfSides = (4*g) + (2*(p-1))
            SidePairings = {}
            for k in range(NumOfSides):
                    SidePairings[k] = Mobius_CP.MobiusTransitivity().MobiusMatrixz1z2z3Tow1w2w3(numpy.cos(2*((f(k)+1)%NumOfSides)*numpy.pi/NumOfSides)+numpy.sin(2*((f(k)+1)%NumOfSides)*numpy.pi/NumOfSides)*(1j),
                                numpy.cos(2*((f(k))%NumOfSides)*numpy.pi/NumOfSides)+numpy.sin(2*((f(k))%NumOfSides)*numpy.pi/NumOfSides)*(1j),
                                midpoints[f(k)],
                                numpy.cos(2*k*numpy.pi/NumOfSides)+numpy.sin(2*k*numpy.pi/NumOfSides)*(1j),
                                numpy.cos(2*(k+1)*numpy.pi/NumOfSides)+numpy.sin(2*(k+1)*numpy.pi/NumOfSides)*(1j),
                                midpoints[k])
                    #print(Mobius_CP.MobiusAssocToMatrix().isParEllHypLox(SidePairings[k][0,0],SidePairings[k][0,1],SidePairings[k][1,0],SidePairings[k][1,1]))
            return SidePairings
                    
    def PDOrbitOfVertexUnderSpecificCombinatorialSidePairing(self,genus,numberOfPunctures):
        g, p = genus, numberOfPunctures
        if g == 0 and p < 3:
            pass
        elif g == 1 and p == 0:
            pass
        else:
            NumOfSides = (4*g) + (2*(p-1))
            f = self.PDSpecificCombinatorialSidePairing(g,p)
            def orbitOfVertex(k):
                orbit = [k]
                j = (f(k) + 1) % NumOfSides
                while j != k:
                    orbit.append(j)
                    j = (f(j) + 1) % NumOfSides
                return orbit
            return orbitOfVertex
        
    def PDVertexOrbitsUnderSpecificCombinatorialSidePairing(self,genus,numberOfPunctures):
        g, p = genus, numberOfPunctures
        if g == 0 and p < 3:
            pass
        elif g == 1 and p == 0:
            pass
        else:
            OrbitFunction = self.PDOrbitOfVertexUnderSpecificCombinatorialSidePairing(g,p)
            NumOfSides = (4*g) + (2*(p-1))
            vertices = [k for k in range(NumOfSides)]
            orbits = []
            while len(vertices) > 0:
                vertex = vertices[0]
                orbit = OrbitFunction(vertex)
                orbits.append(orbit)
                for l in orbit:
                    vertices.remove(l)
            return orbits
    
    def PDCheckIfTheGroupIsFuchsianForSpecificSidePairing(self,genus,numberOfPunctures):
        g, p = genus, numberOfPunctures
        if g == 0 and p < 3:
            pass
        elif g == 1 and p == 0:
            pass
        else:
            sidePairings = self.PDSidePairingsOfSpecificIdealPolygon(g,p)
            orbits = self.PDVertexOrbitsUnderSpecificCombinatorialSidePairing(g,p)
            for orbit in orbits:
                transformation = sidePairings[orbit[0]]
                j = 1
                while j < len(orbit):
                    transformation = sidePairings[orbit[j]] * transformation
                    j = j+1
                print(Mobius_CP.MobiusAssocToMatrix().isParEllHypLox(transformation[0,0],transformation[0,1],transformation[1,0],transformation[1,1]))
                
        
                
            
            
           
            
            
            
        
    
        
    