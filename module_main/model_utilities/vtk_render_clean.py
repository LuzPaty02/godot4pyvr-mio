import numpy as np
import cv2
import vtk
from vtk.util import numpy_support
from vtk.util.numpy_support import vtk_to_numpy
from vtkmodules.vtkCommonColor import vtkNamedColors
import module_main.model_utilities.transformations as tf
from liegroups import SE3

colors = vtkNamedColors()

def inside_test(a, b, len_x):
    pl_1 = np.array(a)
    pl_2 = np.array(b)
    d = np.linalg.norm(pl_2-pl_1)
    print(d)
    if d < len_x:
        return True
    else:
        return False


class RenderUtils:
    """
    Every object needed to render a robot shape
    """

    def __init__(self, model):
        """
        Initialization
        :param model: instance of robot_model
        """
        self.model = model
        self.n_sections = self.model.nsections
        self.points = []
        self.lines = []
        self.linesource = []
        self.filter = []
        self.tubeMapper = []
        self.tubeActor = []

        for i in range(self.n_sections):
            points = vtk.vtkPoints()
            points.SetNumberOfPoints(2)
            points.SetPoint(0, 0, 0, 0)
            points.SetPoint(1, 0, 0, 1)
            self.points.append(points)

            lines = vtk.vtkCellArray()
            lines.InsertNextCell(2)
            lines.InsertCellPoint(0)
            lines.InsertCellPoint(1)
            self.lines.append(lines)

            linesource = vtk.vtkPolyData()
            linesource.SetPoints(self.points[i])
            linesource.SetLines(self.lines[i])
            self.linesource.append(linesource)

            filter = vtk.vtkTubeFilter()
            filter.SetNumberOfSides(8)
            filter.SetRadius(self.model.diameters[i]/2.0)
            filter.SetInputData(self.linesource[i])
            self.filter.append(filter)

            # Map texture coordinates
            map_to_tube = vtk.vtkTextureMapToCylinder()
            map_to_tube.SetInputConnection(filter.GetOutputPort())
            map_to_tube.PreventSeamOn()

            tubeMapper = vtk.vtkPolyDataMapper()
            tubeMapper.SetInputConnection(self.filter[i].GetOutputPort())
            # tubeMapper.SetInputConnection(map_to_tube.GetOutputPort())
            tubeMapper.Update()
            self.tubeMapper.append(tubeMapper)

            tubeActor = vtk.vtkActor()
            tubeActor.SetMapper(self.tubeMapper[i])
            tubeActor.GetProperty().SetColor(colors.GetColor3d("White"))
            # tubeActor.GetProperty().SetOpacity(0.5)
            # tubeActor.SetTexture(texture)
            self.tubeActor.append(tubeActor)

    def render_arm(self, shapes):
        """
        Helper function to render arms, given a multi-array with shape info
        :param shapes: shape multi-array
        :return:
        """

        for i in range(len(shapes)):

            # Create a list of homogeneous coordinates and transform it with T
            shape = np.asarray(shapes[i])
            N = shape.shape[0]
            s = np.concatenate((shape, np.ones((shape.shape[0], 1))), axis=1)

            # Legacy code to not have to rewrite below
            T = np.eye(4)
            shape_trans = np.dot(s, T.T)

            # For each section, apply coordinates to the corresponding actor
            if N > 2:
                self.points[i].SetNumberOfPoints(N)
                self.lines[i].Reset()
                self.lines[i].InsertNextCell(N)

                for j in range(0, N):
                    self.points[i].SetPoint(j,
                                            shape_trans[j, 0],
                                            shape_trans[j, 1],
                                            shape_trans[j, 2])
                    self.lines[i].InsertCellPoint(j)

                self.linesource[i].SetPoints(self.points[i])
                self.linesource[i].SetLines(self.lines[i])
                self.tubeMapper[i].Update()


class render_shape():
    """
    VTK rendering for continuum robots
    """

    def __init__(self, w, h, robot_model):
        """
        Initialization
        :param w: width of image
        :param h: height
        :param robot_model: robot model
        """
        self.w = w
        self.h = h
        self.initCam = False

        # Create and initialize renderer utils
        self.robot_model = robot_model
        self.lRender = RenderUtils(robot_model.robleft)
        self.rRender = RenderUtils(robot_model.robright)

        # # Add a sphere to represent the (fixed) scene
        # # Position of the sphere wrt the fixed frame,
        # # i.e. initial camera position
        # self.sphereCenter = np.array([0, 0, 70])
        # self.sphereSource = vtk.vtkSphereSource()
        # self.sphereSource.SetCenter(0, 0, 70)
        # self.sphereSource.SetRadius(5.0)
        # self.sphereMapper = vtk.vtkPolyDataMapper()
        # self.sphereMapper.SetInputConnection(self.sphereSource.GetOutputPort())
        # self.sphereactor = vtk.vtkActor()
        # self.sphereactor.SetMapper(self.sphereMapper)

        # At first update we will take the endoscope tip pose as orld frame
        self.initEndPose = None

        # Initialize the renderer and window
        self.renderer = vtk.vtkRenderer()
        self.renWin = vtk.vtkRenderWindow()
        self.renWin.SetOffScreenRendering(1)

        # # Add actors AFTER rendering initialization
        # self.renderer.AddActor(self.sphereactor)

        for actor in self.lRender.tubeActor:
            self.renderer.AddActor(actor)
        for actor in self.rRender.tubeActor:
            self.renderer.AddActor(actor)

        # self.renderer.SetBackground(0.1, 0.2, 0.4)
        self.renderer.SetBackground(colors.GetColor3d("White"))
        self.renderer.ResetCamera()

        self.renWin.AddRenderer(self.renderer)
        self.renWin.SetSize(self.w, self.h)

        self.renWin.Render()

    def setCamProperties(self, f, c, K=None):
        self.f = f
        self.c = c

    def render_stras_arms(
       self, shapes_l, shapes_r, end_pose, Tr_e, Tl_e, Tr_0, Tl_0, Te):
        """
        Render the geometry of the STRAS robot
        :param shapes_l:an array of arrays, each row being 3D coordinates of
                         a section of the left robot
        :param shapes_r:an array of arrays, each row being 3D coordinates of
                         a section of the left robot
        :param end_pose:Homogeneous transformation matrix repr the camera pose
        :return:a numpy array with the rendered robot in the camera image plane
        """

        # Render arms with the given shapes arrays
        self.rRender.render_arm(shapes_r)
        self.lRender.render_arm(shapes_l)

        # At first iteration,  current endoscope tip pose as world frame
        if self.initEndPose is None:
            self.initEndPose = end_pose

        # # Render the fixed sphere. We don't move the camera, we move the
        # # environment relative to it, which simulates the endoscope moving
        # tipPose = self.initEndPose.dot(end_pose.inv()).as_matrix()\
        #     @ tf.translation_matrix(self.sphereCenter)
        # self.sphereSource.SetCenter(tipPose[0, 3],
        #                             tipPose[1, 3],
        #                             tipPose[2, 3])
        # self.sphereMapper.Update()

        # Once all positions are set, render the scene using the active camera
        cam = self.renderer.GetActiveCamera()

        # If the camera is not properly initialized, do it here.
        # Parameters are set in such a way that VTK renders using
        # the pinhole model in the "cam" object.
        if not self.initCam:
            near = 0.1
            far = 300.0
            cam.SetClippingRange(near, far)
            cam.SetPosition(0, 0, 0)
            cam.SetFocalPoint(0, 0, 1)
            cam.SetViewUp(0, -1, 0)

            wcx = -2 * (self.c[0] - self.w / 2.0) / self.w
            wcy = 2 * (self.c[1] - self.h / 2.0) / self.h
            cam.SetWindowCenter(wcx, wcy)
            angle = 180 / np.pi * 2.0 * np.arctan2(self.h / 2.0, self.f[1])
            cam.SetViewAngle(angle)

            m = np.eye(4)
            aspect = self.f[1] / self.f[0]
            m[0, 0] = 1.0 / aspect
            t = vtk.vtkTransform()
            t.SetMatrix(m.flatten())
            cam.SetUserTransform(t)
            self.initCam = True

        # Render the scene and export it to an openCV/numpy array to return it
        self.renWin.Render()

        winToIm = vtk.vtkWindowToImageFilter()
        winToIm.SetInput(self.renWin)
        winToIm.Update()
        vtk_image = winToIm.GetOutput()

        width, height, _ = vtk_image.GetDimensions()

        vtk_array = vtk_image.GetPointData().GetScalars()
        components = vtk_array.GetNumberOfComponents()

        arr = cv2.flip(
            numpy_support.vtk_to_numpy(vtk_array).reshape(
                height, width, components), 0)
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

        return arr
