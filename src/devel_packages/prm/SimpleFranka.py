import numpy as np
import math
import time
import random

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

PAYLOAD_LWH = [0.33, 0.33, 0.06] # dimensions of payload box (Length, Width, Height) in metres


class SimpleFrankArm:
    
    def __init__(self, arm_number=1):
        # Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
        if arm_number == 1:
            self.Rdesc=[
                [0, 0, 0, 0., 0, 0.333], # From robot base to joint1
                [-np.pi/2, 0, 0, 0, 0, 0],
                [np.pi/2, 0, 0, 0, -0.316, 0],
                [np.pi/2, 0, 0, 0.0825, 0, 0],
                [-np.pi/2, 0, 0, -0.0825, 0.384, 0],
                [np.pi/2, 0, 0, 0, 0, 0],
                [np.pi/2, 0, 0, 0.088, 0, 0],
                [0, 0, 0, 0, 0, 0.107 + 0.14] # From joint5 to end-effector center
                ]
        else:
            self.Rdesc=[
                [0, 0, 0, 0., 0, 0.333], # From robot base to joint1
                [-np.pi/2, 0, 0, 0, 0, 0],
                [np.pi/2, 0, 0, 0, -0.316, 0],
                [np.pi/2, 0, 0, 0.0825, 0, 0],
                [-np.pi/2, 0, 0, -0.0825, 0.384, 0],
                [np.pi/2, 0, 0, 0, 0, 0],
                [np.pi/2, 0, 0, 0.088, 0, 0],
                [0, 0, 0, 0, 0, 0.107 + 0.125] # From joint5 to end-effector center
                ]
        
        #Define the axis of rotation for each joint 
        self.axis=[
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1],
                [0, 0, 1]
                ]

        #Set base coordinate frame as identity - NOTE: don't change
        self.Tbase= [[1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]]
        
        #Initialize matrices - NOTE: don't change this part
        self.Tlink=[] #Transforms for each link (const)
        self.Tjoint=[] #Transforms for each joint (init eye)
        self.Tcurr=[] #Coordinate frame of current (init eye)
        for i in range(len(self.Rdesc)):
            self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
            self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
            self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

        self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])  

        self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])					

        # initialize Jacobian matrix
        self.J=np.zeros((6,7))
        
        self.q=[0.,0.,0.,0.,0.,0.,0.]
        self.ForwardKin([0.,0.,0.,0.,0.,0.,0.])

        # joint limits		
        self.qmin=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973] # NOTE-does not include grippers
        self.qmax=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973] # NOTE-does not include grippers

        # Robot collision blocks descriptors: (base frame, (rpy xyz), lwh 
        if arm_number == 1:
            self.Cidx = [1, 1, 1, 1, 1, 3, 4, 5, 5, 5, 7, 7, 8] # Joint frame ID for each collision block
        else:
            self.Cidx = [1, 1, 1, 1, 1, 3, 4, 5, 5, 5, 7, 7] # Joint frame ID for each collision block

        # (rpy xyz) poses of the robot arm blocks
        if arm_number == 1:
            self.Cdesc=[
                [0, 0, 0, -.04, 0, -0.283],
                [0, 0, 0, -0.009, 0, -0.183],
                [0.6259876, 0, 0, 0, -0.032, -0.082],
                [0, 0, 0, -0.008, 0, 0],
                [0.6259876, 0, 0, 0, .042, .067],
                [0, 0, 0, 0.00687, 0, -0.139],
                [-(np.pi/2), 0, 0, -0.008, 0.004, 0],
                [-2.8938194, 0, 0, 0.00422, 0.05367, -0.121],
                [0, 0, 0, 0.00422,  0.00367, -0.263],
                [0, 0, 0, 0.00328, 0.0176, -0.0055],
                [-np.pi, 0, 0, -0.0136, 0.0092, 0.0083],
                [0, 0, -0.7853982, 0.0136,  -0.0092,  0.1457 + (0.14 - 0.107)],
                [0, 0, -np.pi/4, 0, 0, PAYLOAD_LWH[2]/2 - 0.015] # payload block, -1.5 cm to account for suction gripper compression
            ]
        else:
            self.Cdesc=[
                [0, 0, 0, -.04, 0, -0.283],
                [0, 0, 0, -0.009, 0, -0.183],
                [0.6259876, 0, 0, 0, -0.032, -0.082],
                [0, 0, 0, -0.008, 0, 0],
                [0.6259876, 0, 0, 0, .042, .067],
                [0, 0, 0, 0.00687, 0, -0.139],
                [-(np.pi/2), 0, 0, -0.008, 0.004, 0],
                [-2.8938194, 0, 0, 0.00422, 0.05367, -0.121],
                [0, 0, 0, 0.00422,  0.00367, -0.263],
                [0, 0, 0, 0.00328, 0.0176, -0.0055],
                [-np.pi, 0, 0, -0.0136, 0.0092, 0.0083],
                [0, 0, -0.7853982, 0.0136,  -0.0092,  0.1457 + (0.125 - 0.107)]
            ]
        
        # dimensions of robot arm blocks (Length, Width, Height)
        if arm_number == 1:
            self.Cdim=[
                [0.23, 0.2, 0.1],
                [0.13, 0.12, 0.1], 
                [0.12, 0.1, 0.2],
                [0.15, 0.27, 0.11],
                [0.12, 0.1, 0.2],
                [0.13, 0.12, 0.25],
                [0.13, 0.23, 0.15],
                [0.12, 0.12, 0.4],
                [0.12, 0.12, 0.25],
                [0.13, 0.23, 0.12],
                [0.12, 0.12, 0.2],
                [0.06, 0.10, 0.14],
                [PAYLOAD_LWH[0], PAYLOAD_LWH[1], PAYLOAD_LWH[2]] # payload block
            ]
        else:		
            self.Cdim=[
                [0.23, 0.2, 0.1],
                [0.13, 0.12, 0.1], 
                [0.12, 0.1, 0.2],
                [0.15, 0.27, 0.11],
                [0.12, 0.1, 0.2],
                [0.13, 0.12, 0.25],
                [0.13, 0.23, 0.15],
                [0.12, 0.12, 0.4],
                [0.12, 0.12, 0.25],
                [0.13, 0.23, 0.12],
                [0.12, 0.12, 0.2],
                [0.06, 0.12, 0.15]
            ]

        self.Tblock=[] #Transforms for each arm block
        self.Tcoll=[]  #Coordinate frame of current collision block

        self.Cpoints=[]
        self.Caxes=[]

        # initialize collision block transforms
        for i in range(len(self.Cdesc)):
            self.Tblock.append(rt.rpyxyz2H(self.Cdesc[i][0:3],self.Cdesc[i][3:6]))
            self.Tcoll.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
            
            self.Cpoints.append(np.zeros((3,4)))
            self.Caxes.append(np.zeros((3,3)))

    def ForwardKin(self,ang):
        '''
        inputs: joint angles
        outputs: joint transforms for each joint, Jacobian matrix
        '''
        self.q[0:-1]=ang
        
        #Compute current joint and end effector coordinate frames (self.Tjoint). Remember that not all joints rotate about the z axis!
        for i in range(len(self.Rdesc)):
            self.Tjoint[i]=[[math.cos(self.q[i]),-math.sin(self.q[i]),0,0],[math.sin(self.q[i]),math.cos(self.q[i]),0,0],[0,0,1,0],[0,0,0,1]]

            if i == 0:
                self.Tcurr[i]=np.matmul(self.Tlink[i],self.Tjoint[i])
            else:
                self.Tcurr[i]=np.matmul(np.matmul(self.Tcurr[i-1],self.Tlink[i]),self.Tjoint[i])
        
        # Compute Jacobian matrix		
        for i in range(len(self.Tcurr)-1):
            p=self.Tcurr[-1][0:3,3]-self.Tcurr[i][0:3,3]
            ax_of_rotation = np.nonzero(self.axis[i])[0][0]	
            a=self.Tcurr[i][0:3,ax_of_rotation]*np.sign(self.axis[i][ax_of_rotation])
            self.J[0:3,i]=np.cross(a,p)
            self.J[3:7,i]=a
            
        return self.Tcurr, self.J

    def IterInvKin(self,ang,TGoal,x_eps=1e-3, r_eps=1e-3):
        '''
        inputs: starting joint angles (ang), target end effector pose (TGoal)

        outputs: computed joint angles to achieve desired end effector pose, 
        Error in your IK solution compared to the desired target
        '''	

        W = np.eye(7)
        W[2,2] = 100
        W[3,3] = 100
        W[6,6] = 100
        C = np.eye(6)*1e6
        C[3,3] = 1000
        C[4,4] = 1000
        C[5,5] = 1000

        self.ForwardKin(ang)
        
        Err=[0,0,0,0,0,0] # error in position and orientation, initialized to 0

        for s in range(10000):
            #Compute rotation error
            Rerr = np.matmul(TGoal[:3,:3], np.transpose(self.Tcurr[-1][:3,:3]))
            rErrAxis, rErrAng = rt.R2axisang(Rerr)
            rErr = [rErrAxis[0]*rErrAng, rErrAxis[1]*rErrAng, rErrAxis[2]*rErrAng]
            X_err = TGoal[0:3,3] - self.Tcurr[-1][0:3, 3]

            Err[0:3] = X_err
            Err[3:6] = rErr

            # if norm of the error is below thresholds, then exit
            if np.linalg.norm(Err[0:3]) <= x_eps and np.linalg.norm(Err[3:6]) <= r_eps:
                break

            #Reduce Step Size driven by Error
            if rErrAng>0.1:
                rErrAng=0.1

            if rErrAng<-0.1:
                rErrAng=-0.1

            rErr = [rErrAxis[0]*rErrAng, rErrAxis[1]*rErrAng, rErrAxis[2]*rErrAng]

            if np.linalg.norm(X_err) > 0.01:
                X_err = (X_err/np.linalg.norm(X_err))*0.01

            Err[0:3] = X_err
            Err[3:6] = rErr
    
            J_pound = np.linalg.inv(W)@(self.J.T)@np.linalg.inv(self.J@(np.linalg.inv(W)@self.J.T) + np.linalg.inv(C))

            delta_q = J_pound@Err

            self.q[0:-1] += delta_q
            
            self.ForwardKin(self.q[0:-1]) #Recompute forward kinematics for new angles

        #print("Iterations: ", s)

        return self.q[0:-1], Err

    def PlotSkeleton(self,ang):
        self.ForwardKin(ang)

        # Create figure
        fig =plt.figure()
        ax = fig.add_subplot(111,projection='3d')

        #Draw links along coordinate frames 
        for i in range(len(self.Tcurr)):
            ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
            if i == 0:
                ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='b')
            else:
                ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

        #Format axes and display
        ax.axis('equal')
        ax.set(xlim=(-1., 1.), ylim=(-1., 1.), zlim=(0,1.))
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.show()

    def SampleRobotConfig(self):
        q=[]
        for i in range(7):
            q.append(self.qmin[i]+(self.qmax[i]-self.qmin[i])*random.random())
        return q


    def CompCollisionBlockPoints(self,ang, Cidx=None, Cdesc=None, Cdim=None):
        self.ForwardKin(ang)

        # TODO-Compute current collision boxes for arm 
        # Tcoll - stores the transform for each collision block of the robot links
        # Cpoints - stores the center and 8 corner points of the collision block
        # Caxes - stores the axes of the collision block

        # Note: you can use rt.BlockDesc2Points to get the cpoints and axes for each collision block
        for i, link in enumerate(self.Cidx):
            self.Tcoll[i]= np.matmul(self.Tcurr[link-1], self.Tblock[i])
            self.Cpoints[i], self.Caxes[i] = rt.BlockDesc2Points(self.Tcoll[i], self.Cdim[i])
        
    def AttachPayload(self, l, b, h):
        """Add a collision box at the end-effector to represent a held object."""
        self.Cidx.append(8)                          # end-effector frame (Tcurr[7])
        desc = [0, 0, -np.pi/4, 0, 0, h/2 - 0.015]        # centered at end-effector origin, +1.5cm to account for suction gripper compression
        self.Cdesc.append(desc)
        self.Cdim.append([l, b, h])
        self.Tblock.append(rt.rpyxyz2H(desc[0:3], desc[3:6]))
        self.Tcoll.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
        self.Cpoints.append(np.zeros((3,4)))
        self.Caxes.append(np.zeros((3,3)))

    def RemovePayload(self):
        """Remove the payload collision box added by AttachPayload."""
        if len(self.Cidx) > 12:  # 12 is the number of base arm blocks
            self.Cidx.pop()
            self.Cdesc.pop()
            self.Cdim.pop()
            self.Tblock.pop()
            self.Tcoll.pop()
            self.Cpoints.pop()
            self.Caxes.pop()

    def DetectCollision(self, ang, pointsObs, axesObs):		
        self.CompCollisionBlockPoints(ang)

        for i in range(len(self.Cpoints)):
            for j in range(len(pointsObs)):
                if rt.CheckBoxBoxCollision(self.Cpoints[i],self.Caxes[i],pointsObs[j], axesObs[j]):
                    return True
        return False


    def DetectCollisionEdge(self, ang1, ang2, pointsObs, axesObs):
        for s in np.linspace(0,1,10): #The number of steps along the edge may need to be adapted
            ang= [ang1[k]+s*(ang2[k]-ang1[k]) for k in range(len(ang1))] 	
            self.CompCollisionBlockPoints(ang)
            for i in range(len(self.Cpoints)):
                for j in range(len(pointsObs)):
                    if rt.CheckBoxBoxCollision(self.Cpoints[i],self.Caxes[i],pointsObs[j], axesObs[j]):
                        return True
        return False

    def PlotCollisionBlockPoints(self,ang,pointsObs=[]):
        self.CompCollisionBlockPoints(ang)
        fig =plt.figure()
        ax = fig.add_subplot(111,projection='3d')


        #Draw links along coordinate frames 
        for i in range(len(self.Tcurr)):
            ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
            if i == 0:
                ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='k')
            else:
                ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], 
                    [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], 
                    [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

        for b in range(len(self.Cpoints)):
            for i in range(1,9): #TODO might have to change the 9 to 5?
                for j in range(i,9):
                    ax.plot([self.Cpoints[b][i][0],  self.Cpoints[b][j][0]], 
                        [self.Cpoints[b][i][1],  self.Cpoints[b][j][1]], 
                        [self.Cpoints[b][i][2],  self.Cpoints[b][j][2]], c='b')
                
        for b in range(len(pointsObs)):
            for i in range(1,9):
                for j in range(i,9):
                    ax.plot([pointsObs[b][i][0],  pointsObs[b][j][0]], 
                        [pointsObs[b][i][1],  pointsObs[b][j][1]], 
                        [pointsObs[b][i][2],  pointsObs[b][j][2]], c='r')


        #Format axes and display
        ax.set(xlim=(-0.6, .6), ylim=(-0.6, 0.6), zlim=(0,1.2))
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        plt.show()
        return fig, ax
    
if __name__ == "__main__":
    arm = FrankArm()
    arm.PlotCollisionBlockPoints([0,0,0,0,0,0,0])