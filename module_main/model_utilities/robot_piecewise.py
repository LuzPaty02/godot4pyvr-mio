# -*- coding: utf-8 -*-
"""
Created on Tue Sep 22 10:00:55 2015

@author: CH182482
"""

#==============================================================================
# Imports 
#==============================================================================

# numpy
import numpy as np
import module_main.model_utilities.transformations as tf
import scipy.linalg as la

#==============================================================================
# Function defines
#==============================================================================

def rotate(R,x):
    '''
    Returns the rotated 3-vector by rotation matrix R

    Input R is homogeneous matrix
    Input x is a 3-vector    
    '''
    x_ = np.append(x,[1])
    return R.dot(x_.reshape(4,))[:3]





class Robot_piecewise_curvature:
    '''    
    Create a Piecewise robot with 2 tubes structure
    
    Any combination of V and F tubes can be achieved through setCurvature
    '''
    
    def __init__(self, n_sections):
    
        self.n_sec = n_sections
        self.r = np.zeros(n_sections)
        self.l = np.zeros(n_sections)
        
        
    def setCurvature(self, r):
    
        if len(r) < self.n_sec:
            print("Not enough radii provided, n_tubes = ", self.n_sec)
            raise ValueError
        self.r = np.asarray(r)
        
        
    def setLength(self, l):
        if len(l) < self.n_sec:
            print("Not enough radii provided, n_tubes = ", self.n_sec)
            raise ValueError
        self.l = np.asarray(l)
        
    def setOrigin(self, P, nz, nx):
        self.P = np.asarray(P, dtype=float)
        self.z = np.asarray(nz, dtype=float).copy()
        self.z /= np.linalg.norm(self.z)
        self.x = np.asarray(nx, dtype=float).copy()
        self.x /= np.linalg.norm(self.x)
        self.y = np.cross(self.z, self.x)
        
    def getOrigin(self):
        return (self.P, self.x, self.y, self.z)
        
    def getCurvature(self):
        return self.r
        
    def getLength(self):
        return self.l
        
    def getTipPose(self, d, th, tipOnly = False):
        '''
        Compute tip position and orientation by assuming constant piecewise curvature
        Robot has an intial position at P, with insertion along z and zero rotation = xz plane
        
        Arguments:
            d = lengths of tubes (array, self.n_sec values)
            th = rotation of tubes (array, self.n_sec values)
            
        Returns:
            shape: array with the whole shape
            tangent: tangent at the tip
            dest_P: array containing shape of sections
            reachable: boolean with reachable position
        '''
        
        P = []
        tan = []
        norm = []
        
        try:            
            (dest_P, dest_tan, dest_norm) = self.computeCircArc(self.P,self.z, self.x, th[0], self.r[0], d[0], tipOnly)
        except:
            raise
        
        P.append(dest_P[:-1,:])
        tan.append(dest_tan)
        norm.append(dest_norm)
        
        for i in range(1,self.n_sec):
            try:            
                (dest_P, dest_tan, dest_norm) = self.computeCircArc(dest_P[-1],tan[i-1][-1], norm[i-1], th[i], self.r[i], d[i], tipOnly)
            except:
                raise
            P.append(dest_P)
            tan.append(dest_tan)
            norm.append(dest_norm)
            
        shape = np.vstack(P)
        tangent = np.vstack(tan)
        
        return (shape, tangent, P, True)
  
        
    def computeCircArc(self, start_P, start_tan, start_n, th, r, l,
                       tipOnly=False, n_samples=None):
        '''
        Compute pose of the tip of a circular arc
        
        Arguments:
            start_P: start position
            start_tan: tangent at start
            start_n: reference normal before rotation
            th: rotation angle of the arc around start_tan, wrt start_n
            r: radius of curvature
            l: insertion length
            tipOnly: boolean, compute only tip position and not whole shape
            
        Returns:
            dest_P: points along the arc
            dest_tan: tangent at the end
            dest_norm: normal to the arc plane
        '''
        
        start_P = np.asarray(start_P, dtype=float)
        start_tan = np.asarray(start_tan, dtype=float)
        start_n = np.asarray(start_n, dtype=float)
        if abs(r) >= 1e12 or np.isinf(r):
            if tipOnly:
                dest_P = start_P + float(l) * start_tan
                return dest_P, start_tan.copy(), np.cross(start_tan, start_n)
            count = max(2, int(n_samples or 10))
            distances = np.linspace(0.0, l, num=count)
            dest_P = start_P + distances[:, None] * start_tan[None, :]
            dest_tan = np.repeat(start_tan[None, :], count, axis=0)
            return dest_P, dest_tan, np.cross(start_tan, start_n)

        R_th1 = tf.rotation_matrix(th,start_tan)
        u = rotate(R_th1, start_n) 
        n = np.cross(u,start_tan)
        
        el_per_mm = 2
        num_ = max(2, int(n_samples) if n_samples is not None
               else int(max(el_per_mm * l, 10)))
        
        l_ = np.linspace(0,l,num=num_)

        if (tipOnly):
            l_ = l
            alpha = l*1.0 / r
            a = l_*1.0 / r
    
            center = start_P - r*u
    
            dest_P = center + r * ( np.cos(a) * np.array(u).reshape(1,3) + np.sin(a) * np.cross(n,u).reshape(1,3) ) 
            dest_tan = rotate( tf.rotation_matrix(alpha,n), start_tan )
            dest_norm = np.cross(dest_tan,n)
            return(dest_P, dest_tan, dest_norm)
            
        alpha = l*1.0 / r
        a = l_*1.0 / r

        center = start_P - r*u

        dest_P = center + r * ( np.cos(a).reshape(a.shape[0],1) * np.array(u).reshape(1,3) + np.sin(a).reshape(a.shape[0],1) * np.cross(n,u).reshape(1,3) )

        dest_tan = []
        for al in a:
            dest_tan.append(rotate(tf.rotation_matrix(al,n),start_tan))
        dest_tan = np.array(dest_tan)
        dest_norm = np.cross(dest_tan[-1],n)
        
        return(dest_P, dest_tan, dest_norm)

    def getTipSpeed(self,q,n_sections,tipOnly=False):
        """
        Return the speed of the tip in ? frame to define, camera or robot
        :param q: 
        :param n_sections: 
        :param tipOnly: 
        :return: 
        """

        dt = 0.1
        q_dot = np.gradient(q,dt)

        J = self.computeJacobian(q,n_sections)

        X_dot = np.dot(J,q_dot)
        return X_dot

    def computeJacobian(self,n_sections):
        """
        Calculate the jacobian matrix of a n-sections continuum robot 
        :param phi: rotation of the whole section
        :param r: curvature radius of section
        :param l: length of section
        :param n_sections: number of sections
        :return: Jacobian matrix of the robot
        """

        L = np.array([20, 45, 60])
        phi = np.array([45, 90, 120])
        s = np.array([5, 10, 15])
        d = np.array([10.0, 10.0, 11.0])

        T = np.zeros((n_sections, 4, 4))
        Adj = np.zeros((n_sections, 6, 6))
        J = np.zeros((n_sections, 6, 3))
        Js_rob = np.zeros((6, 3 * n_sections))

        twist_rot = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])

        for ii in range(n_sections):

            k = L[ii] / (s[ii] * d[ii])

            twist_inp = np.array([[0, 0, k, 0], [0, 0, 0, 0], [-k, 0, 0, 1], [0, 0, 0, 0]])

            T[ii, ...] = la.expm(twist_rot * phi[ii]).dot(la.expm(twist_inp * L[ii]))
            Tf = np.eye(4)
            for ij in range(ii):
                Tf = np.dot(Tf, T[ij])
            R = Tf[0:3, 0:3]
            p = Tf[0:3, 3]
            Adj[ii, 0:3, 0:3] = R
            Adj[ii, 3:6, 3:6] = R
            Adj[ii, 3:6, 0:3] = np.dot(p * np.eye(3), R)

            J[ii, ...] = [[np.cos(phi[ii]) * (np.cos(k * s[ii]) - 1) / (k * k), 0, 0],
                          [np.sin(phi[ii]) * (np.cos(k * s[ii]) - 1) / (k * k), 0, 0],
                          [-(np.sin(k * s[ii]) - k * s[ii]) / (k * k), 0, 1],
                          [-s[ii] * np.sin(phi[ii]), 0, -k * np.sin(phi[ii])],
                          [s[ii] * np.cos(phi[ii]), 0, k * np.cos(phi[ii])],
                          [0, 1, 0]]

            Js_rob[:, ii * 3:((ii + 1) * 3)] = np.dot(Adj[ii, ...], J[ii, ...])

        return Js_rob


        
        
'''
Test code launched if the script is called directly

Note that this code is not launched if the module is imported in another script
'''
if __name__ == '__main__':

    print(0)

    # CTR = Robot_piecewise_curvature(2)
    # CTR.setCurvature([39.572025457,60.2055469603])
    # CTR.setLength([1000,1000])
    # CTR.setOrigin([38.56,-42.23,35.56],[-0.45706417,0.74816542,-0.48097905],[0.88878462,0.45828992,-0.00567915])
    #
    #
    # CTR2 = CTR_piecewise()
    # CTR2.setCurvature(39.572025457,60.2055469603)
    # CTR2.setLength(1000,1000)
    # CTR2.setOrigin([38.56,-42.23,35.56],[-0.45706417,0.74816542,-0.48097905],[0.88878462,0.45828992,-0.00567915])
    #
    # print "Unit testing, going over different configurations"
    # OK = True
    # n = 0
    # tol = 10**(-8)
    #
    #
    #
    # d1 = 12.5
    # d2 = 35.0
    # th1 = np.pi/3.0
    # th2 = np.pi/2.0
    #
    # (shape, tangent, P, reachable) = CTR.getTipPose([d1,d2],[th1,th2])
    # (shape2, tangent2, destP1, destP2, reachable) = CTR2.getTipPose(d1,d2,th1,th2)
    #
    # print P[0][-1], destP1[-1]
    # print P[1][-1], destP2[-1]
    #
    # print shape[-1]
    # print shape2[-1]

#    for d1 in np.linspace(0,50,num=5):
#        for d2 in np.linspace(0,50,num=5):
#            for th1 in np.linspace(0,np.pi/2.0, num=5):
#                for th2 in np.linspace(0,np.pi/2.0, num=5):
#                    (shape, tangent, reachable) = CTR.getTipPose([d1,d2],[th1,th2])
#                    (shape2, tangent2, destP1, dest_P2, reachable) = CTR2.getTipPose(d1,d2,th1,th2)

#                    if not np.allclose(tangent2,tangent, rtol = tol, atol = tol) or not np.allclose(shape,shape2, rtol = tol, atol = tol):
#                        n += 1
#
#    if n>0:
#        print "Mismatch between Circular Arc and Exponential methods in ", n, " configurations"
#    else:
#        print "Good match at tolerance ", tol, " for all ",5*5*5*5,"tested configurations"



# Example of plotting commented afterwards


#    start = time.time()
#
#    for c in np.linspace(40,100,num=10):
#        for d1 in np.linspace(0,50,num=5):
#            for th2 in np.linspace(0,np.pi,num=5):
#                for d2 in np.linspace(0,50,num=5):
#                    CTR.setCurvature(c,30)
#                    (shape, tangent, dest_P1, dest_P2, reachable) = CTR.getTipPose(d1,d2,0.0,th2)
#                    if reachable:
#                        tip.append(shape[-1])
#                        tip_tan.append(tangent)
#
#    end = time.time()
#
#
#
#
#    for i in range(0,len(tip)):
#        axes.quiver(tip[i][0], tip[i][1], tip[i][2], tip_tan[i][0], tip_tan[i][1], tip_tan[i][2], length = 5, colors = [0,1,0])
##    axes.plot(dest_P1[:,0], dest_P1[:,1], dest_P1[:,2], c='b')
##    axes.plot(dest_P2[:,0], dest_P2[:,1], dest_P2[:,2], c='r')
#
#    (P,x,y,z) = CTR.getOrigin()
#    axes.scatter(P[0], P[1], P[2], c='r')
#    axes.quiver(P[0], P[1], P[2],z[0], z[1], z[2], length=10, colors=[1,0,0])
#

#    axes.plot(shape[:,0],shape[:,1],shape[:,2], c='b')
#    axes.plot(shape_w[:,0],shape_w[:,1],shape_w[:,2], c='r')
#
#
#    axisEqual3D(axes)
#    pyplot.show()


