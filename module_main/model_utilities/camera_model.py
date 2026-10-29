import numpy as np
from scipy.io import loadmat
import cv2
import math


class camera_model:
    """
    Class implementing a pinhole camera model with parameters loaded from a
    matlab file.
    It has functions to project a point on an image, rectify an image ...
    """

    def __init__(self, matfile):
        self.matInfo = loadmat(matfile)
        self.f = self.matInfo['fc'].flatten()
        self.c = self.matInfo['cc'].flatten()
        self.imgSize = np.array([self.matInfo['nx'][0, 0],
                                 self.matInfo['ny'][0, 0]])

        self.K = np.array([[self.f[0], 0, self.c[0]],
                           [0, self.f[1], self.c[1]],
                           [0, 0, 1]])
        self.kc = self.matInfo['kc'].flatten()

        imSize = (self.imgSize[0], self.imgSize[1])
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.K,
                                                           self.kc,
                                                           None,
                                                           self.K,
                                                           imSize,
                                                           5)

    def getImSize(self):
        return self.imgSize[0], self.imgSize[1]

    def getF(self):
        return self.f

    def getC(self):
        return self.c

    def rectify_image(self, img):
        return cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)

    def project_point(self, x, y, z):
        vec = np.array([[x], [y], [z]])
        return np.dot(self.K, vec)

    def project_point(self, Point):
        return cv2.projectPoints(Point.reshape(1, 3),
                                 (0, 0, 0),
                                 (0, 0, 0),
                                 self.K,
                                 self.kc)[0].flatten()

    def interpolate_map(self, x, y):
        xm = int(math.floor(x))
        xM = int(math.ceil(x))
        x_res = xM - x

        ym = int(math.floor(y))
        yM = int(math.ceil(y))
        y_res = yM - y

        undist_m = [self.mapx[ym, xm] + x_res*self.mapx[yM, xM],
                    self.mapy[ym, xm] + y_res*self.mapy[yM, xM]]
        return [self.mapx[xm, ym], self.mapy[xm, ym]]
