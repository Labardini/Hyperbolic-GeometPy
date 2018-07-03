#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 07:54:19 2018

@author: daniellabardini
"""

import numpy



from exception_handling import myInputError
from Maths.CP_Maths import extended_complex_plane_CP


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
myNumpyCosecant = extended_complex_plane_CP.numpyExtendedComplexPlane().myNumpyCosecant
myNumpyCotangent = extended_complex_plane_CP.numpyExtendedComplexPlane().myNumpyCotangent
myarg0To2Pi = extended_complex_plane_CP.numpyExtendedComplexPlane().myarg0To2Pi
####



        
class UHPBasics:
    
    def __init__(self):
        pass
    
    def isInUHP(self,complexP):
        P = extendedValue(complexP)
        if P != oo and P.imag > 0:
            answer = True
        else:
            answer = False
        return answer
    
    def isIdealPoint(self,complexP):
        P = extendedValue(complexP)
        if P == oo or P.imag == 0:
            answer = True
        else:
            answer = False
        return answer
    
    def isInUHPBar(self,complexP):
        if self.isInUHP(complexP) == True or self.isIdealPoint(complexP) == True:
            answer = True
        else:
            answer = False
        return answer
        
    def areVertAlignedInUHP(self,hPointP,hPointQ):
        P, Q = extendedValue(hPointP), extendedValue(hPointQ)
        if self.isInUHPBar(P) == True and self.isInUHPBar(Q) == True and P != Q:
            if P == oo or Q == oo:
                answer = True
            elif P.real == Q.real:
                answer = True
            else:
                answer = False
            return answer
        else:
            pass
            #raise myInputError(str(P)+','+str(Q),"Both points must belong to UHPBar and they must be distinct")

    def UHPNorm(self,point,vector): # IN THE UPPER HALF PLANE
        P = numpy.complex(point)
        V = numpy.complex(vector)
        result =   numpy.sqrt( (numpy.real(V))**2 + (numpy.imag(V))**2 )  / numpy.imag(P)
        return result

    def UHPDist(self,pointP,pointQ): # UPPER HALF PLANE
        P = numpy.complex(pointP)
        Q = numpy.complex(pointQ)
        P1 = numpy.real(P)
        P2 = numpy.imag(P)
        Q1 = numpy.real(Q)
        Q2 = numpy.imag(Q)
        dist_expression = numpy.arccosh(  1 + ( ( (Q1-P1)**2 + (Q2-P2)**2 ) / (2*P2*Q2) )  )
        return dist_expression
    
            
    def eCenterAndRadiusNonVertGeodesicThroughPAndQ(self,hpointP,hpointQ):
        P, Q = extendedValue(hpointP), extendedValue(hpointQ)
        if self.areVertAlignedInUHP(P,Q) == False:
            normal = (Q-P).imag + (P-Q).real*(1j)
            midpoint = (P+Q)/2
            eCenter = midpoint + (-(midpoint.imag)/(normal.imag))*(normal)
            eRadius = numpy.absolute(P-eCenter)
            return [eCenter,eRadius]
        else:
            raise myInputError(str(P)+','+str(Q),"The points must be distinct, belong to the UHP, and not be vertically alligned")
            
    def eCenterAndRadiusH_Circle(self,hcenter,hradius): # IN THE UPPER HALF PLANE
        Hcenter = numpy.complex(hcenter)
        Hradius = numpy.real(hradius)            
        x0 = Hcenter.real
        x1 = Hcenter.imag
        euclidean_center_x = x0
        euclidean_center_y = x1*(numpy.cosh(Hradius))
        #euclidean_center = Point(euclidean_center_x, euclidean_center_y)
        euclidean_radius = x1*(numpy.sinh(Hradius))
        return [ numpy.complex(euclidean_center_x + euclidean_center_y*(1j)), numpy.real(euclidean_radius)]

    def UHPGeodesicSegment_rcostrsint(self,startpoint,endpoint):
        A = extendedValue(startpoint)
        B = extendedValue(endpoint)
        if self.areVertAlignedInUHP(A,B) == False:
            X = self.eCenterAndRadiusNonVertGeodesicThroughPAndQ(A,B)
            translated_startpoint = A - (X[0])
            translated_endpoint = B - (X[0])
            arg_trans_startpoint = numpy.angle(translated_startpoint)
            arg_trans_endpoint = numpy.angle(translated_endpoint)
            #distance1 = self.UHPDist(startpoint,endpoint)
            #distance2 = numpy.abs(numpy.log( ( myNumpyCosecant(arg_trans_endpoint) - myNumpyCotangent(arg_trans_endpoint) ) / ( myNumpyCosecant(arg_trans_startpoint) - myNumpyCotangent(arg_trans_startpoint) ) ))#self.UHPDist(A,B)
            interval = numpy.linspace(arg_trans_startpoint,arg_trans_endpoint)
        else:
            #distance1 = self.UHPDist(startpoint,endpoint)
            #distance2 = numpy.abs(numpy.log(numpy.imag(B) / numpy.imag(A)))
            interval = numpy.linspace(A.imag,B.imag)
        #print("distances=",distance1,distance2)
        def parametrized_curve(t):
            if self.areVertAlignedInUHP(A,B) == False:
                e_center_of_geodesic = X[0]
                e_radius_of_geodesic = X[1]
                parametrization = e_center_of_geodesic + e_radius_of_geodesic*( numpy.cos(t) + numpy.sin(t)*(1j) )
            else:
                parametrization = numpy.real(A) + t*(1j)
            return parametrization
        return parametrized_curve(interval)
 

######
###### FOR POLYGONS IN UHP:
       
###### TANGENT UNIT VECTOR OF A GIVEN DIRECTED GEODESIC AT A GIVEN POINT

    def tangent_unit(self,point1,point2):
        z = extendedValue(point1)
        w = extendedValue(point2)
        if z.real != w.real:
            center = e_circumcenter_and_radius(z,w,numpy.conjugate(w))[0][0]+e_circumcenter_and_radius(z,w,numpy.conjugate(w))[0][1]*(1j)
            connecting_vector = z-center
            orthogonal_vector = connecting_vector*(-1j)
            unit_vector =  numpy.sign(w.real-z.real)*orthogonal_vector/self.UHPNorm(z,orthogonal_vector)
        else:
            connecting_z_and_w = w-z
            unit_vector= connecting_z_and_w/self.UHPNorm(z,connecting_z_and_w)
        return unit_vector


    def tangent_unit_Hbar(self,point1,point2):
        if point1==point2:
            pass
        else:
            z = extendedValue(point1)
            w = extendedValue(point2)
            if z != oo and 0 < z.imag and w != oo and 0 < w.imag:
                result = self.tangent_unit(point1,point2)
            if z != oo and z.imag == 0:
                result = 1j
            if z == oo:
                result = -1j
            if w == oo:
                result = 1j
            if w!=oo and w.imag == 0 and z.imag != 0 and z.imag != oo:
                if z.real==w.real:
                    result = -1j
                else:
                    center = e_circumcenter_and_radius(z,w,numpy.conjugate(z))[0][0]+e_circumcenter_and_radius(z,w,numpy.conjugate(z))[0][1]*(1j)
                    connecting_vector = z-center
                    orthogonal_vector = connecting_vector*(-1j)
                    result =  numpy.sign(w.real-z.real)*orthogonal_vector/self.UHPNorm(point1,orthogonal_vector)
            return result
    


#### CYCLIC ORDERING OF THREE POINTS AROUND A FIXED BASEPOINT

##def cyclic_order_counter_clockwise(basepoint,point1,point2,point3):
##    vector1=unit_vector(basepoint,point1)
##    vector2=unit_vector(basepoint,point2)
##    vector3=unit_vector(basepoint,point3)
##    difference1_of_arguments_in_radians = arg_radians(vector2)-arg_radians(vector1)
##    difference2_of_arguments_in_radians = arg_radians(vector3)-arg_radians(vector1)
##    if difference2_of_arguments_in_radians>difference1_of_arguments_in_radians:
##        order = [point1,point2,point3]
##    if difference2_of_arguments_in_radians<difference1_of_arguments_in_radians:
##        order=[point1,point3,point2]

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
        #print(ordered_list_of_arguments)
        cyclic_order = []
        for t in ordered_list_of_arguments:
            for point in remaining_vertices:
                vector = self.tangent_unit_Hbar(basepoint,point)
                difference_of_arguments_in_radians = myarg0To2Pi(vector/reference_vector)#numpy.angle(vector/reference_vector)
                if t == difference_of_arguments_in_radians:
                    cyclic_order.append(point)        
        # THIS DOESN'T WORK NICELY IF AN ARGUMENT APPEARS TWICE: cyclic_order = [vertices[list_of_arguments.index(t)] for t in  ordered_list_of_arguments]
        return cyclic_order
            

    def is_point_in_h_convex_hull(self,point,points):
        remaining_vertices = [k for k in points]
        for k in points:
            if remaining_vertices.count(k)>1:
                remaining_vertices.remove(k)
    ##    if point in remaining_vertices:
    ##        remaining_vertices.remove(point)
        cyclic_order = self.cyclic_order_counter_clockwise(point,remaining_vertices)
        first_element = cyclic_order[0]
        cyclic_order.append(first_element)
        #print(cyclic_order)
        arguments = []
        for i in range(0,len(cyclic_order)-1,1):
            #arguments.append( arg_0_360 (c_aDivByb(tangent_unit_Hbar(point,cyclic_order[i+1]), tangent_unit_Hbar(point,cyclic_order[i]) )))
            arguments.append( myarg0To2Pi( self.tangent_unit_Hbar(point,cyclic_order[i+1])/ self.tangent_unit_Hbar(point,cyclic_order[i]) ))
        if max(arguments) > numpy.pi or point.imag==0:
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
        if max(arguments) >= numpy.pi or point.imag==0:
                       result = False
        else:
            result = True
        return result


        
###### DECIDING COLINEARITY IN UHP

    def are_three_points_collinear(self,point1,point2,point3):
        if point1==point2 or point1==point3 or  point2==point3:
            pass
        elif isooInArgs(point1,point2,point3) == True:
            finitePoints = [k for k in [point1,point2,point3] if k != oo]
            if finitePoints[0].real == finitePoints[1].real:
                answer = True
            else:
                answer = False
        elif point1.real == point2.real == point3.real:
            answer = True
        #elif c_mult(c_aMinusb(point2,point1), [c_aMinusb(point3,point1)[0], -(c_aMinusb(point3,point1)[1])])[1] != 0:
         #   answer = 'No'
        elif e_circumcenter_and_radius(point1,point2,point3)[0][1] !=0:
            answer = False
        elif e_circumcenter_and_radius(point1,point2,point3)[0][1] ==0:
            answer = True
        return answer
            
    
        
                
    def find_verts_of_h_convex_hull(self,points):
    ##    point0=points[0]
    ##    point1=points[1]
    ##    #vertex_set = [point0,point1]
    ##    extended_vertex_set = [point0,point1]
    ##    i=2
    ##    while i<len(points):
    ##        if are_three_points_collinear(point0,point1,points[i]) == 'No':
    ##            point2 = points[i]
    ##            break
    ##        else:
    ##            i = i+1
    ##    vertex_set.append(point2)
    ##    for j in range(2,i+1,1):
    ##        extended_vertex_set.append(points[j])
    ##    test_list=[points[t] for t in range(i+1,len(points),1)]
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
        if self.areVertAlignedInUHP(A,B) == False:
            X = self.eCenterAndRadiusNonVertGeodesicThroughPAndQ(A,B)
            translated_startpoint = A - (X[0])
            translated_endpoint = B - (X[0])
            arg_trans_startpoint = numpy.angle(translated_startpoint)
            arg_trans_endpoint = numpy.angle(translated_endpoint)
            t = (arg_trans_endpoint + arg_trans_startpoint)/2
            point = X[0] + X[1]*( numpy.cos(t) + numpy.sin(t)*(1j) )
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
            if ell == oo or ell.imag == 0:
                ideal_vert.append(ell)
        finite_vert=[]
        for ell in vert:
            if ell != oo and 0 <ell.imag:
                finite_vert.append(ell)
        basepoint = self.somePointOnGeodesicSegment(vert[0],vert[1])
        if finite_vert != []:
            initial = finite_vert[0]
            #finite_vert.remove(basepoint)
            finite_vert.extend(ideal_vert)
            vertices = self.cyclic_order_counter_clockwise(basepoint,finite_vert)
            #vertices.insert(0,basepoint)
            vertices.append(initial)
        if finite_vert == []:
            vertices = self.cyclic_order_counter_clockwise((1j),vert)
            p0=vertices[0]
            vertices.append(p0)
        return vertices

        
        

    



        







    
class UHPGeodesicMotion:
    
    def __init__(self):
        pass
    
    def UHPGeodesicSegmentParamByArcLength(self,startpoint,endpoint): 
        A = extendedValue(startpoint)
        B = extendedValue(endpoint)
        if UHPBasics().areVertAlignedInUHP(A,B) == False:
            X = UHPBasics().eCenterAndRadiusNonVertGeodesicThroughPAndQ(A,B)
            translated_startpoint = A - (X[0])
            translated_endpoint = B - (X[0])
            arg_trans_startpoint = numpy.angle(translated_startpoint)
            arg_trans_endpoint = numpy.angle(translated_endpoint)
            distance = UHPBasics().UHPDist(startpoint,endpoint)
            interval_for_s = numpy.log( ( myNumpyCosecant(arg_trans_endpoint) - myNumpyCotangent(arg_trans_endpoint) ) / ( myNumpyCosecant(arg_trans_startpoint) - myNumpyCotangent(arg_trans_startpoint) ) )#self.UHPDist(A,B)
            #print(distance,interval_for_s)
        else:
            interval_for_s = numpy.log(numpy.imag(B) / numpy.imag(A))
        left_to_right_interval_for_s = numpy.abs(interval_for_s)
        def parametrized_curve(time_s):
            ARC_LENGTH_PARAMETER = numpy.real(time_s)
            if UHPBasics().areVertAlignedInUHP(A,B) == False:
                e_center_of_geodesic = X[0]
                e_radius_of_geodesic = X[1]
                t  = 2 * ( numpy.arctan( (numpy.tan( arg_trans_startpoint / 2 )) * numpy.exp(ARC_LENGTH_PARAMETER) ) )
                parametrization = e_center_of_geodesic + e_radius_of_geodesic*( numpy.cos(t) + numpy.sin(t)*(1j) )
            else:
                t = numpy.imag(A)*( numpy.exp(ARC_LENGTH_PARAMETER) )
                parametrization = numpy.real(A) + t*(1j)
            return parametrization
        def reoriented_parametrization(increasing_s):
            if interval_for_s >= 0:
                result = parametrized_curve(increasing_s)
            if interval_for_s < 0:
                result = parametrized_curve(-increasing_s)
            return result
        return [reoriented_parametrization, left_to_right_interval_for_s]
    
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
            if V.real > 0:
                Q = eCenter + (eRadius * ( (numpy.cos(angle/2)) + (numpy.sin(angle/2)*(1j)) ))
            if V.real < 0:
                Q = eCenter + (eRadius * ( (numpy.cos((numpy.pi+angle)/2)) + (numpy.sin((numpy.pi+angle)/2)*(1j)) ))
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

    
    
    