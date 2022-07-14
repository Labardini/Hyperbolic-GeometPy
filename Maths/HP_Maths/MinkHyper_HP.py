#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 12 15:19:38 2022

@author: caesar
"""

import numpy as np


class Hyperboloid:

    def __init__(self):
        pass

    def minkowski(self, pt, u, v):
        x, y, z = pt[0], pt[1], pt[2]
        ux, uy = u[0], u[1]
        vx, vy = v[0], v[1]

        # Basis for TpM

        # b1 = [1, 0, x/z]
        # b2 = [0, 1, y/z]

        # We use this basis to identify TpM with R2 so as to compute <u,v>

        uu = [ux, uy, ux*x/z+uy*y/z]
        vv = [vx, vy, vx*x/z+vy*y/z]

        prod = uu[0]*vv[0]+uu[1]*vv[1]-uu[2]*vv[2]
        return prod


    def geodesic(self, pt, tan_vector, t):
        mag = np.sqrt(self.minkowski(pt, tan_vector, tan_vector))
        #vz = vx*pt[0]/pt[2] + vy*pt[1]/pt[2]
        a = np.cosh(mag*t)
        b = np.sinh(mag*t)/mag
        return [a*pt[i] + b*tan_vector[i] for i in range(0, 3)]

        # x = np.cosh(mag*t)*pt[0] + np.sinh(mag*t)*tan_vector[0]/mag
        # y = np.cosh(mag*t)*pt[1] + np.sinh(mag*t)*tan_vector[1]/mag
        # z = np.cosh(mag*t)*pt[2] + np.sinh(mag*t)*tan_vector[2]/mag
        # return [x, y, z]

    def mapToDisk(self, pt):
        d = pt[2]+1
        x = pt[0]/d
        y = pt[1]/d
        return [x, y]

    def mapToHyp(self, pt):
        mag2 = pt[0]**2 + pt[1]**2
        s = 1/(1-mag2)
        point = [s*2*pt[0], s*2*pt[1], s*(1 + mag2)]
        return point

    def pointFromXY(self, px, py):
        pz = np.sqrt(px*px+py*py+1)
        return [px, py, pz]


    def tangentVector(self, pt, vec): # We identify R2 with the tangent plane of M at p
        x = pt[0]
        y = pt[1]
        z = pt[2]
        vx = vec[0]
        vy = vec[1]
        v = [vx, vy, vx*x/z+vy*y/z]
        return v
