from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
#from first import main as R_knee_pitch
import ctypes
import _ctypes
import pygame
import sys
import math
import time 
from naoqi import ALProxy
if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
              pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()
        robotIp = "192.168.0.102"
        self.motionProxy = ALProxy("ALMotion", robotIp,9559)
        self.postureProxy = ALProxy("ALRobotPosture", robotIp,9559)
        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)
        
        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None

    def foottest(self):
        
        legName  = ["LLeg"]
        X        = 0.2
        Y        = 0.1
        Theta    = 0.3
        footSteps = [[X, Y, Theta]]
        timeList = [0.6]
       
        clearExisting = False
        self.motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)







    #centering the coordinates system
    def Build_Coor(self,joints,joint0,joint1,joint2, joint3):
         
        ShoulderRight = [joints[joint0].Position.x,joints[joint0].Position.y,joints[joint0].Position.z]
        ShoulderLeft = [joints[joint1].Position.x,joints[joint1].Position.y,joints[joint1].Position.z]
        SpineBase = [joints[joint2].Position.x,joints[joint2].Position.y,joints[joint2].Position.z]  
        SpineShoulder = [joints[joint3].Position.x,joints[joint3].Position.y,joints[joint3].Position.z]
        #confirm X axis:increasing X direction is from ShoulderLeft to ShoulderRight
        Vector_Shoulder_Right_Left =[ ShoulderLeft[0] - ShoulderRight[0], ShoulderLeft[1] - ShoulderRight[1], ShoulderLeft[2] - ShoulderRight[2]]
        #confirm Y axis: increasing Y direction is from SpineShoulder to SpineBase
        Vector_Shoulder_Spine_Base = [SpineBase[0]-SpineShoulder[0],SpineBase[1]-SpineShoulder[1], SpineBase[2]-SpineShoulder[2]]
        #confirm Z axis
        k = Vector_Shoulder_Right_Left[0]/Vector_Shoulder_Spine_Base[0]
        New_Vector = [(SpineBase[0]-SpineShoulder[0])*k,(SpineBase[1]-SpineShoulder[1])*k, (SpineBase[2]-SpineShoulder[2])*k]
        Sub = [Vector_Shoulder_Right_Left[0]-New_Vector[0], Vector_Shoulder_Right_Left[1]-New_Vector[1], Vector_Shoulder_Right_Left[2]-New_Vector[2]]
        Z = - Sub[1]/Sub[2]
        Y = 1
        x = (-Vector_Shoulder_Right_Left[1] - Vector_Shoulder_Right_Left[2]* Z) / Vector_Shoulder_Right_Left[0]
        coor_z = [x,Y,Z]
        #Vector_Mul = Vector_Shoulder_Right_Left[0]*Vector_Shoulder_Spine_Base[0] + Vector_Shoulder_Right_Left[1]*Vector_Shoulder_Spine_Base[1] + Vector_Shoulder_Right_Left[2]*Vector_Shoulder_Spine_Base[2]
        #a = coor_z[0]*Vector_Shoulder_Spine_Base[0] + coor_z[1]*Vector_Shoulder_Spine_Base[1] + coor_z[2]*Vector_Shoulder_Spine_Base[2]
        #b = coor_z[0]*Vector_Shoulder_Right_Left[0] + coor_z[1]*Vector_Shoulder_Right_Left[1] + coor_z[2]*Vector_Shoulder_Right_Left[2]
        #print "Vector_Mul %s" % Vector_Mul
        #print "coor_z =  %s" % coor_z
        #print "a = %s" % a
        #print "b = %s" % b
        return Vector_Shoulder_Right_Left, Vector_Shoulder_Spine_Base, coor_z
    def Cal_Joint_angel_RightArm(self,joints,joint0,joint1,joint2,coor):
        coor_x = coor[0]
        coor_x1 = [-1*x for x in coor[0]]
        coor_y = coor[1]
        coor_z = coor[2]
        
        Shoulder = [joints[joint0].Position.x,joints[joint0].Position.y,joints[joint0].Position.z]
        Elbow = [joints[joint1].Position.x,joints[joint1].Position.y,joints[joint1].Position.z]
        Wrist = [joints[joint2].Position.x,joints[joint2].Position.y,joints[joint2].Position.z]
      
        #Point_Z_coor = [coor_z[0]+Shoulder[0], coor_z[1]+Shoulder[1], coor_z[2]+Shoulder[2]]
      
        Vector_Shoulder_Elbow =[ Elbow[0] - Shoulder[0], Elbow[1] - Shoulder[1], Elbow[2] - Shoulder[2]]
        Vector_Shoulder_Wrist = [Wrist[0] - Shoulder[0], Wrist[1] - Shoulder[1], Wrist[2] - Shoulder[2]]
        Vector_Elbow_Wrist = [Wrist[0] - Elbow[0], Wrist[1] - Elbow[1], Wrist[2] - Elbow[2]]
        
        Elbow_Wrist_Dis = math.sqrt(Vector_Elbow_Wrist[0]**2+ Vector_Elbow_Wrist[1]**2 + Vector_Elbow_Wrist[2]**2)
        #print Vector_Elbow_Wrist
        Vector_ShEl_y_Mul = coor_y[0]*Vector_Shoulder_Elbow[0] + coor_y[1]*Vector_Shoulder_Elbow[1] + coor_y[2]*Vector_Shoulder_Elbow[2]
        Vector_ShEl_y_Dis = math.sqrt((Vector_Shoulder_Elbow[0]**2+Vector_Shoulder_Elbow[1]**2+Vector_Shoulder_Elbow[2]**2)*(coor_y[0]**2 + coor_y[1]**2 + coor_y[2]**2))
        Shoulder_Elbow_y_costheta = Vector_ShEl_y_Mul / Vector_ShEl_y_Dis
        Shoulder_Pitch = math.acos(Shoulder_Elbow_y_costheta) - math.pi/2
        
        #tanTheta = (Shoulder[0]-Point_Z_coor[0])/(Elbow[2]-Shoulder[2])
        costheta = (Vector_Shoulder_Elbow[0]*coor_x1[0] +Vector_Shoulder_Elbow[1]*coor_x1[1]+ Vector_Shoulder_Elbow[2]*coor_x1[2])/math.sqrt((Vector_Shoulder_Elbow[0]**2+Vector_Shoulder_Elbow[1]**2+Vector_Shoulder_Elbow[2]**2)*(coor_x1[0]**2+coor_x1[1]**2+coor_x1[2]**2))
        theta = math.acos(costheta)
        Shoulder_Roll =  theta-math.pi/2
        #print "Shoulder_Roll = %s" % Shoulder_Roll
        #tanTheta = (Vector_Shouder_Elbow[0]-coor_z[0])/(Vector_Shouder_Elbow[2])
        #cosTheta = Vector_Shouder_Elbow[0]/math.sqrt(Vector_Shouder_Elbow[0]**2+Vector_Shouder_Elbow[1]**2+Vector_Shouder_Elbow[2]**2)
        #Shouder_Roll = math.acos(cosTheta) - math.pi/2
        #tanTheta = Vector_Shouder_Elbow[1]/Vector_Shouder_Elbow[2]
        #Shouder_Roll =    - math.atan(tanTheta) 
       
       # print "theta = %s" % theta
        Vector_Mul = Vector_Shoulder_Elbow[0]*Vector_Elbow_Wrist[0] + Vector_Shoulder_Elbow[1]*Vector_Elbow_Wrist[1] + Vector_Shoulder_Elbow[2]*Vector_Elbow_Wrist[2]
        Vector_Dis = math.sqrt((Vector_Shoulder_Elbow[0]**2+Vector_Shoulder_Elbow[1]**2+Vector_Shoulder_Elbow[2]**2)*(Vector_Elbow_Wrist[0]**2 + Vector_Elbow_Wrist[1]**2 + Vector_Elbow_Wrist[2]**2))
        Shoulder_Elbow_costheta = Vector_Mul / Vector_Dis
        Elbow_Roll =  math.acos(Shoulder_Elbow_costheta)
        
        
        #Vector_ElWr_y_Mul = coor_y[0]*Vector_Elbow_Wrist[0] + coor_y[1]*Vector_Elbow_Wrist[1] + coor_y[2]*Vector_Elbow_Wrist[2]
        #Vector_ElWr_y_Dis = math.sqrt((Vector_Elbow_Wrist[0]**2+Vector_Elbow_Wrist[1]**2+Vector_Elbow_Wrist[2]**2)*(coor_y[0]**2 + coor_y[1]**2 + coor_y[2]**2))
        #Elbow_Wrist_y_costheta = Vector_ShEl_y_Mul / Vector_ShEl_y_Dis
        #Elbow_Yaw = math.pi - math.acos(Elbow_Wrist_y_costheta)    
        Elbow_Yaw = math.asin((Wrist[1]-Elbow[1])/Elbow_Wrist_Dis)
        #print"Elbow_Yaw = %s" % Elbow_Yaw
        #Shouder_Elbow_tantheta = Vector_Shouder_Wrist[1]/Vector_Shouder_Wrist[2]
        #Elbow_Yaw = - math.atan(Shouder_Elbow_tantheta)
        if Shoulder_Pitch > 2.08:
            Shoulder_Pitch = 2.08
        if Shoulder_Pitch < -2.0857:
            Shoulder_Pitch = -2.08
        if Shoulder_Roll > 0.31:
            Shoulder_Roll = 0.31
        if Shoulder_Roll < -1.3265:
            Shoulder_Roll = -1.32
        if Elbow_Roll > 1.54:
            Elbow_Roll = 1.54
        if Elbow_Roll < 0.0349:
            Elbow_Roll = 0.0349
        if Elbow_Yaw > 2.08:
            Elbow_Yaw = 2.08
        if Elbow_Yaw < -2.0857:
            Elbow_Yaw = -2.08
        #print "shoulder_pitch = %s" % Shoulder_Pitch
        #print "shoudler_Roll = %s" % Shoulder_Roll
        #print "Elbow_Roll = %s" % Elbow_Roll
        #print "Elbow_Yaw = %s" % Elbow_Yaw 
        return Shoulder_Pitch ,Shoulder_Roll ,Elbow_Roll,Elbow_Yaw + Shoulder_Pitch

    def Cal_Joint_angel_RightLeg(self,joints,joint0,joint1,joint2,coor):
        ccoor_x = coor[0]
        coor_x1 = [-1*x for x in coor[0]]
        coor_y = coor[1]
        coor_y1 =  [-1*y for y in coor[1]]
        coor_z = coor[2]
        
        Hip = [joints[joint0].Position.x,joints[joint0].Position.y,joints[joint0].Position.z]
        Knee = [joints[joint1].Position.x,joints[joint1].Position.y,joints[joint1].Position.z]
        Ankle = [joints[joint2].Position.x,joints[joint2].Position.y,joints[joint2].Position.z]
      
        #Point_Z_coor = [coor_z[0]+Shoulder[0], coor_z[1]+Shoulder[1], coor_z[2]+Shoulder[2]]
      
        Vector_Hip_Knee =[ Knee[0] - Hip[0], Knee[1] - Hip[1], Knee[2] - Hip[2]]
        Vector_Hip_Ankle = [Ankle[0] - Hip[0], Ankle[1] - Hip[1], Ankle[2] - Hip[2]]
        Vector_Knee_Ankle = [Ankle[0] - Knee[0], Ankle[1] - Knee[1], Ankle[2] - Knee[2]]
        
        Knee_Ankle_Dis = math.sqrt(Vector_Knee_Ankle[0]**2+ Vector_Knee_Ankle[1]**2 + Vector_Knee_Ankle[2]**2)
        #print Vector_Elbow_Wrist
        Vector_HiKn_y_Mul = coor_y1[0]*Vector_Hip_Knee[0] + coor_y1[1]*Vector_Hip_Knee[1] + coor_y1[2]*Vector_Hip_Knee[2]
        Vector_HiKn_y_Dis = math.sqrt((Vector_Hip_Knee[0]**2+Vector_Hip_Knee[1]**2+Vector_Hip_Knee[2]**2)*(coor_y1[0]**2 + coor_y1[1]**2 + coor_y1[2]**2))
        Hip_Knee_y_costheta = Vector_HiKn_y_Mul/ Vector_HiKn_y_Dis
        Hip_Pitch = math.acos(Hip_Knee_y_costheta)
        if Hip[2] > Knee[2]:
            Hip_Pitch = -Hip_Pitch
        
        #tanTheta = (Shoulder[0]-Point_Z_coor[0])/(Elbow[2]-Shoulder[2])
        costheta = (Vector_Hip_Knee[0]*coor_x1[0] +Vector_Hip_Knee[1]*coor_x1[1]+ Vector_Hip_Knee[2]*coor_x1[2])/math.sqrt((Vector_Hip_Knee[0]**2+Vector_Hip_Knee[1]**2+Vector_Hip_Knee[2]**2)*(coor_x1[0]**2+coor_x1[1]**2+coor_x1[2]**2))
        theta = math.acos(costheta)
        Hip_Roll =  theta-math.pi/2
        #print "Shoulder_Roll = %s" % Shoulder_Roll
        #tanTheta = (Vector_Shouder_Elbow[0]-coor_z[0])/(Vector_Shouder_Elbow[2])
        #cosTheta = Vector_Shouder_Elbow[0]/math.sqrt(Vector_Shouder_Elbow[0]**2+Vector_Shouder_Elbow[1]**2+Vector_Shouder_Elbow[2]**2)
        #Shouder_Roll = math.acos(cosTheta) - math.pi/2
        #tanTheta = Vector_Shouder_Elbow[1]/Vector_Shouder_Elbow[2]
        #Shouder_Roll =    - math.atan(tanTheta) 
       
       # print "theta = %s" % theta
        Vector_Mul = Vector_Hip_Knee[0]*Vector_Knee_Ankle[0] + Vector_Hip_Knee[1]*Vector_Knee_Ankle[1] + Vector_Hip_Knee[2]*Vector_Knee_Ankle[2]
        Vector_Dis = math.sqrt((Vector_Hip_Knee[0]**2+Vector_Hip_Knee[1]**2+Vector_Hip_Knee[2]**2)*(Vector_Knee_Ankle[0]**2 + Vector_Knee_Ankle[1]**2 + Vector_Knee_Ankle[2]**2))
        Hip_Knee_costheta = Vector_Mul / Vector_Dis
        Knee_Pitch =  math.acos(Hip_Knee_costheta)
        if Hip_Pitch > 0.484:
            Hip_Pitch = 0.484
        if Hip_Pitch < -1.53:
            Hip_Pitch = -1.53
        if Hip_Roll > 0.378:
            Hip_Roll = 0.378
        if Hip_Roll < -0.785:
            Hip_Roll = -0.785
        if Knee_Pitch > 2.12:
            Knee_Pitch = 2.12
        if Knee_Pitch < -0.112:
            Knee_Pitch = 0.112

        #print 'Hip_Roll =', Hip_Roll
        return Hip_Pitch ,Hip_Roll ,Knee_Pitch
        


    def Cal_Joint_angle_LeftArm(self,joints,joint0,joint1,joint2,coor):
        coor_x = coor[0]
        coor_x1 = [-1*x for x in coor[0]]
        coor_y = coor[1]
        coor_z = coor[2]

        Shoulder = [joints[joint0].Position.x, joints[joint0].Position.y, joints[joint0].Position.z]
        Elbow = [joints[joint1].Position.x, joints[joint1].Position.y, joints[joint1].Position.z]
        Wrist = [joints[joint2].Position.x, joints[joint2].Position.y, joints[joint2].Position.z]

        #Point_Z_coor = [coor_z[0] + Shoulder[0], coor_z[1] + Shoulder[1], coor_z[2] + Shoulder[2]]

        Vector_Shoulder_Elbow = [Elbow[0] - Shoulder[0], Elbow[1] - Shoulder[1], Elbow[2] - Shoulder[2]]

        Vector_Shoulder_Wrist = [Wrist[0] - Shoulder[0], Wrist[1] - Shoulder[1], Wrist[2] - Shoulder[2]]
        Vector_Elbow_Wrist = [Wrist[0] - Elbow[0], Wrist[1] - Elbow[1], Wrist[2] - Elbow[2]]

        Elbow_Wrist_Dis = math.sqrt(Vector_Elbow_Wrist[0] ** 2 + Vector_Elbow_Wrist[1] ** 2 + Vector_Elbow_Wrist[2] ** 2)
        # print Vector_Elbow_Wrist
        Vector_ShEl_y_Mul = coor_y[0] * Vector_Shoulder_Elbow[0] + coor_y[1] * Vector_Shoulder_Elbow[1] + coor_y[2] * Vector_Shoulder_Elbow[2]
        Vector_ShEl_y_Dis = math.sqrt((Vector_Shoulder_Elbow[0] ** 2 + Vector_Shoulder_Elbow[1] ** 2 + Vector_Shoulder_Elbow[2] ** 2) * (coor_y[0] ** 2 + coor_y[1] ** 2 + coor_y[2] ** 2))
        Shoulder_Elbow_y_costheta = Vector_ShEl_y_Mul / Vector_ShEl_y_Dis
        Shoulder_Pitch = math.acos(Shoulder_Elbow_y_costheta) - math.pi / 2

        # tanTheta = (Shoulder[0]-Point_Z_coor[0])/(Elbow[2]-Shoulder[2])
        costheta = (Vector_Shoulder_Elbow[0] * coor_x1[0] + Vector_Shoulder_Elbow[1] * coor_x1[1]+Vector_Shoulder_Elbow[2] * coor_x1[2]) / math.sqrt((Vector_Shoulder_Elbow[0] ** 2 +Vector_Shoulder_Elbow[1] ** 2+ Vector_Shoulder_Elbow[2] ** 2) * (coor_x1[0] ** 2 +coor_x1[1] ** 2+ coor_x1[2] ** 2))
        theta = math.acos(costheta)
        Shoulder_Roll = theta -math.pi/2
        # tanTheta = (Vector_Shouder_Elbow[0]-coor_z[0])/(Vector_Shouder_Elbow[2])
        # cosTheta = Vector_Shouder_Elbow[0]/math.sqrt(Vector_Shouder_Elbow[0]**2+Vector_Shouder_Elbow[1]**2+Vector_Shouder_Elbow[2]**2)
        # Shouder_Roll = math.acos(cosTheta) - math.pi/2
        # tanTheta = Vector_Shouder_Elbow[1]/Vector_Shouder_Elbow[2]
        # Shouder_Roll =    - math.atan(tanTheta)

        # print "theta = %s" % theta
        Vector_Mul = Vector_Shoulder_Elbow[0] * Vector_Elbow_Wrist[0] + Vector_Shoulder_Elbow[1] * Vector_Elbow_Wrist[1] + Vector_Shoulder_Elbow[2] * Vector_Elbow_Wrist[2]
        Vector_Dis = math.sqrt((Vector_Shoulder_Elbow[0] ** 2 + Vector_Shoulder_Elbow[1] ** 2 + Vector_Shoulder_Elbow[2] ** 2) * (Vector_Elbow_Wrist[0] ** 2 + Vector_Elbow_Wrist[1] ** 2 + Vector_Elbow_Wrist[2] ** 2))
        Shoulder_Elbow_costheta = Vector_Mul / Vector_Dis
        Elbow_Roll = - math.acos(Shoulder_Elbow_costheta)

        # Vector_ElWr_y_Mul = coor_y[0]*Vector_Elbow_Wrist[0] + coor_y[1]*Vector_Elbow_Wrist[1] + coor_y[2]*Vector_Elbow_Wrist[2]
        # Vector_ElWr_y_Dis = math.sqrt((Vector_Elbow_Wrist[0]**2+Vector_Elbow_Wrist[1]**2+Vector_Elbow_Wrist[2]**2)*(coor_y[0]**2 + coor_y[1]**2 + coor_y[2]**2))
        # Elbow_Wrist_y_costheta = Vector_ShEl_y_Mul / Vector_ShEl_y_Dis
        # Elbow_Yaw = math.pi - math.acos(Elbow_Wrist_y_costheta)
        #costheta1 = (Vector_Elbow_Wrist[0] * coor_x[0] +Vector_Elbow_Wrist[1] * coor_x[1]+Vector_Elbow_Wrist[2] * coor_x[2]) / math.sqrt((Vector_Elbow_Wrist[0] ** 2 +Vector_Elbow_Wrist[1] ** 2+ Vector_Elbow_Wrist[2] ** 2) * (coor_x[0] ** 2 +coor_x[1] ** 2+ coor_x[2] ** 2))
        #Elbow_Yaw = math.acos(costheta1) - math.pi
        if -0.523 <= Elbow_Roll <=-0.017:
            Elbow_Yaw = -1.186
        else:
            Elbow_Yaw =  -math.asin((Wrist[1]-Elbow[1])/Elbow_Wrist_Dis)
        #print"Elbow_Yaw = %s" % Elbow_Yaw
        # Shouder_Elbow_tantheta = Vector_Shouder_Wrist[1]/Vector_Shouder_Wrist[2]
        # Elbow_Yaw = - math.atan(Shouder_Elbow_tantheta)
        if Shoulder_Pitch >2.08:
            Shoulder_Pitch = 2.08
        if Shoulder_Pitch < -2.0857:
            Shoulder_Pitch = -2.08
        if Shoulder_Roll < -0.31:
            Shoulder_Roll = -0.31
        if Shoulder_Roll > 1.3:
            Shoulder_Roll = 1.3
        if Elbow_Roll < -1.53:
            Elbow_Roll = -1.53
        if Elbow_Roll > -0.034:
            Elbow_Roll = -0.034
        if Elbow_Yaw > 2:
            Elbow_Yaw = 2
        if Elbow_Yaw < -2:
            Elbow_Yaw = -2
        #print "shoulder_pitch = %s" % Shoulder_Pitch
        #print "Shoulder_Roll = %s" % Shoulder_Roll
        # print "Elbow_Roll = %s" % Elbow_Roll
        #print "Elbow_Yaw = %s" % Elbow_Yaw
        return Shoulder_Pitch, Shoulder_Roll, Elbow_Roll, Elbow_Yaw + Shoulder_Pitch

    #def get_Body_Position(self):



    
    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState
        joint1State = joints[joint1].TrackingState

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)
        if joint0 == PyKinectV2.JointType_Head:
            #print start, end
            theta = (start[0]-end[0])/(end[1]-start[1])
            import math
            #print 180*math.atan(theta)/math.pi

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft)
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight)

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft)

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight)

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft)


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def motion(self,radsLroll,radsRroll,radsLpitch,radsRpitch,radeLroll,radeRroll,radeLyaw,radeRyaw):
        self.motionProxy.post.angleInterpolationWithSpeed("LShoulderRoll",radsLroll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("RShoulderRoll", radsRroll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("LShoulderPitch", radsLpitch, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("RShoulderPitch", radsRpitch, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("LElbowRoll", radeLroll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("RElbowRoll", radeRroll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("LElbowYaw", radeLyaw, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("RElbowYaw", radeRyaw, 0.4)
        #self.motionProxy.post.angleInterpolationWithSpeed("RHipPitch", radhippitch, 0.4)
        #self.motionProxy.post.angleInterpolationWithSpeed("RHipRoll", radhiproll, 0.4)
        #self.motionProxy.post.angleInterpolationWithSpeed("RKneePitch", radkneepitch, 0.4)
    '''def foot(self):
        self.motionProxy.post.angleInterpolationWithSpeed("RHipYawPitch",-0.118, 1)
        self.motionProxy.post.angleInterpolationWithSpeed('RHipRoll',-0.307,1)
        self.motionProxy.post.angleInterpolationWithSpeed('RHipPitch',-0.008,1)
        self.motionProxy.post.angleInterpolationWithSpeed('RAnkleRoll',0.312,1)
        self.motionProxy.post.angleInterpolationWithSpeed('RKneePitch',0.0034,1)
        self.motionProxy.post.angleInterpolationWithSpeed('RAnklePitch',-0.008,1)
        
        self.motionProxy.post.angleInterpolationWithSpeed('LHipYawPitch',-0.118,1)
        self.motionProxy.post.angleInterpolationWithSpeed('LHipRoll',-0.155,1)
        self.motionProxy.post.angleInterpolationWithSpeed('LHipPitch',-0.171,1)
        self.motionProxy.post.angleInterpolationWithSpeed('LAnkleRoll',0.155,1)
        self.motionProxy.post.angleInterpolationWithSpeed('LKneePitch',0.342,1)
        self.motionProxy.post.angleInterpolationWithSpeed('LAnklePitch',-0.148,1)'''


















   def run(self):
        PORT =9559
        ip = '192.168.0.102'
        contsi =0
        replayarray = []
        cnt = 0
        
        # -------- Main Program Loop -----------
        while cnt<75:
            #print("cnt = %s"%cnt)
            #angels = R_knee_pitch(ip,PORT)
            #for i in angels:
                #newamgel = float(i)
                #print newamgel
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            # 
            
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 

                     
                    coor = self.Build_Coor(joints, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineBase)
                    Shoulder_RPitch, Shoulder_RRoll ,Elbow_RRoll, Elbow_RYaw = self.Cal_Joint_angel_RightArm(joints, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_HandRight,coor)
                    Shoulder_LPitch, Shoulder_LRoll, Elbow_LRoll, Elbow_LYaw = self.Cal_Joint_angle_LeftArm(joints, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_HandLeft,coor)
                   # Hip_R_pitch,Hip_R_roll,Knee_R_Pitch  = self.Cal_Joint_angel_RightLeg(joints,PyKinectV2.JointType_HipRight,PyKinectV2.JointType_KneeRight,PyKinectV2.JointType_AnkleRight,coor)
                    if cnt < 65:
                        replayarray.append([Shoulder_LRoll, Shoulder_RRoll, Shoulder_LPitch, Shoulder_RPitch, Elbow_LRoll, Elbow_RRoll, Elbow_LYaw, Elbow_RYaw])
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])
  
            

            
                    self.motion(Shoulder_LRoll, Shoulder_RRoll, Shoulder_LPitch, Shoulder_RPitch, Elbow_LRoll, Elbow_RRoll, Elbow_LYaw, Elbow_RYaw)
                    time.sleep(0.2)
                    cnt += 1
                   
                



                    #joint_points = numpy.ndarray((PyKinectV2.JointType_Count), dtype=numpy.object)
                    #print joints[6].Position.x,joints[6].Position.y,joints[6].Position.z
                    
                    #print Camera_distance
                       
                    # convert joint coordinates to color space 
                    #joint_points = self._kinect.body_joints_to_color_space(joints)
                    
                    #self.draw_body(joints, joint_points, SKELETON_COLORS[i])

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height))
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()
            
            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

           
            # --- Limit to 60 frames per second
            self._clock.tick(60)
            
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()
        time.sleep(0.5)
        memorytouch = ALProxy("ALMemory",ip,PORT)
        touched = 0
        while True:
            if memorytouch.getData("FrontTactilTouched") > 0:
                touched = 1
                break

        time.sleep(0.5)
        lenth = len(replayarray)
        for t in range(lenth):

            Shoulder_LRoll, Shoulder_RRoll, Shoulder_LPitch, Shoulder_RPitch, Elbow_LRoll, Elbow_RRoll, Elbow_LYaw, Elbow_RYaw = replayarray[t]
            self.motion(Shoulder_LRoll, Shoulder_RRoll, Shoulder_LPitch, Shoulder_RPitch, Elbow_LRoll, Elbow_RRoll, Elbow_LYaw, Elbow_RYaw)
            time.sleep(0.2)
        self.motionProxy.rest()
__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime()
game.run()

