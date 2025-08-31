import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
# In order to run the code, type the following in a terminal window: python3 Kennedy_Ian_capstone.py . Ensure that the ModernRobotics library
# is in the directory

def matrixtoRow(T,ee):
    """
    Computes the row vector associated with the transform and the gripper (open/closed) status
        :param T: end effector configuration in the space frame
        :param ee: end effector open/closed status
        :return: row vector with configuration and gripper status
    """
    return np.array([ [T[0,0], T[0,1], T[0,2],T[1,0], T[1,1], T[1,2], T[2,0], T[2,1], T[2,2],T[0,3],T[1,3],T[2,3],ee ]]   )

def rowtoMatrix(r):
    """
    Computes the transform configuration from a row of the trajectory matrix
        :param r: row vector of the 
    """
    T = np.zeros((4,4))
    T[3,3] = 1

    T[0,0] = r[0]
    T[0,1] = r[1]
    T[0,2] = r[2]
    T[1,0] = r[3]
    T[1,1] = r[4]
    T[1,2] = r[5]
    T[2,0] = r[6]
    T[2,1] = r[7]
    T[2,2] = r[8]
    T[0,3] = r[9]
    T[1,3] = r[10]
    T[2,3] = r[11]

    return T



def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
    """
    Computes the end effetor trajectory of the robot end-effector over time in matrix form, with the end effector open/closed status.
        :param Tse_init: initial configuration of the end effector in the space frame
        :param Tsc_init: initial configuration of the cube in the space frame
        :param Tsc_final: final configuration of the cube in the space frame
        :param Tce_grasp: end effector configuration in the cube frame
        :param Tce_standoff: end effector standoff position in the cube frame
        :param k: number of reference configurations per 0.01 seconds
        :return: the Nx13 matrix of the configuration matrix entries and the end effector open close status
    """

    #Starting position
    traj = matrixtoRow(Tse_init,0)
    R = np.array([[-1, 0,  0],
            [0, -1, 0],
            [0, 0,  1]])
    p = np.array([0,0,0])
    T = mr.RpToTrans(R,p)
    p2 = np.array([0,-0.1,0.1])
    R = np.array([[-1, 0,  0],
            [0, -1, 0],
            [0, 0,  1]])
    T2 = mr.RpToTrans(R,p2)
    
    R3 = np.array([[0, 1,  0],
            [-1, 0, 0],
            [0, 0,  1]])
    T3 = mr.RpToTrans(R3, p)
    #Move to initial stand off
    temp = mr.CartesianTrajectory(Tse_init, Tsc_init@Tce_standoff, 10,10*100*k,method=3)

    for i in range(0,len(temp)):
        traj = np.vstack( (traj, matrixtoRow(temp[i],0)   )    )

    #closing
    for i in range(0,100*1*k):
        traj = np.vstack( (traj, matrixtoRow(temp[-1],0)   )    )


    #Move to grasp position
    temp = mr.ScrewTrajectory(Tsc_init@Tce_standoff, Tsc_init@Tce_grasp, 1,1*100*k,method=5)

    for i in range(0,len(temp)):
        traj = np.vstack( (traj, matrixtoRow(temp[i],0)   )    )

    #closing
    for i in range(0,100*1*k):
        traj = np.vstack( (traj, matrixtoRow(temp[-1],1)   )    )

    #Move to initial standoff
    temp = mr.ScrewTrajectory( Tsc_init@Tce_grasp, Tsc_init@Tce_standoff, 1,1*100*k,method=3)

    for i in range(0,len(temp)):
        traj = np.vstack( (traj, matrixtoRow(temp[i],1)   )    )

    #Move to final standoff
    temp = mr.CartesianTrajectory(Tsc_init@Tce_standoff,Tsc_final@Tce_standoff , 10,10*100*k, method=3)
 
    for i in range(0,len(temp)):
        traj = np.vstack( (traj, matrixtoRow(temp[i],1)   )    )

    last_one = rowtoMatrix(traj[-1])

    #Move to drop off position Tsc_final@Tce_standoff
    temp = mr.ScrewTrajectory(last_one,Tsc_final@Tce_grasp , 1,1*100*k, method=3)

    for i in range(0,len(temp)):
        traj = np.vstack( (traj, matrixtoRow(temp[i],1)   )    )

    #opening
    for i in range(0,65*1*k):
        traj = np.vstack( (traj, matrixtoRow(temp[-1],0)   )    )     

    
    #Move to standoff position
    temp = mr.ScrewTrajectory(temp[-1], Tsc_final@Tce_standoff, 1,1*100*k, method=3)


    for i in range(0,len(temp)):
        traj = np.vstack( (traj, matrixtoRow(temp[i],0)   )    )

    return traj

def statetoRow(state, ee):
    """Converts a robot state to a state with gripper information
    :param state: 12 position state of the robot
    :param ee: binary end effector state
    :return: 13 position state of the robot with ee status
    """

    ee_row = ee*np.ones((1,1))

    return np.hstack((state,ee_row))


def NextState(q, u, dt, wdot_max):
    """
    Calculates the next position of the robot given the current position, the control inputs, and the max velocity
    :param q: the current state of the robot
    :param u: the control inputs to the robot
    :param dt: the timestep of the simulation
    :param wdot_max: the max commanded velocity
    :return: the next state of the robot
    """
    
    l = 0.47/2
    w = 0.3/2
    r = 0.0475
    for i in range(0,9):# Check for maximum velocity edge case
        if abs(u[i])>wdot_max:
            u[i] = wdot_max*np.sign(u[i])

    #Compute pseudoinverse of H(0)
    F = np.zeros((3,4))
    F[0,0] = 0.25*r*(-1)/(l+w)
    F[0,1] = 0.25*r*1/(l+w)
    F[0,2] = 0.25*r*1/(l+w)
    F[0,3] = 0.25*r*(-1)/(l+w)
    F[1,0] = 0.25*r*1
    F[1,1] = 0.25*r*1
    F[1,2] = 0.25*r*1
    F[1,3] = 0.25*r*1
    F[2,0] = 0.25*r*(-1)
    F[2,1] = 0.25*r*1
    F[2,2] = 0.25*r*(-1)
    F[2,3] = 0.25*r*1

    wheels = q[8:]
    arm = q[3:8]
    uwheels = u[:4]
    uarm = u[4:]

    #estimate next joint angles
    armnew = arm + dt*uarm

    wheelnew = wheels + dt*uwheels

    # Compute body twist
    Vb = F@(uwheels*dt)
    Vb6 = np.zeros(6)
    Vb6[2] = Vb[0]
    Vb6[3] = Vb[1]
    Vb6[4] = Vb[2]

    Vb6_se3 = mr.VecTose3(Vb6)
    Tbbpr = mr.MatrixExp6(Vb6_se3)
    R, p = mr.TransToRp(Tbbpr)

    dq = np.zeros((3,1))

    #Compute chassis configuration
    if(mr.NearZero(Vb6[2])):
        dq[1,0] = Vb[1]
        dq[2,0] = Vb[2]
    else:
        dq[0,0] = Vb[0]
        dq[1,0] = (Vb[1]*np.sin(Vb[0])+Vb[2]*(np.cos(Vb[0]) - 1))/Vb[0]
        dq[2,0] = (Vb[2]*np.sin(Vb[0]) + Vb[1]*(1-np.cos(Vb[0]) ))/Vb[0]

    phik = q[0]

    phimat = np.zeros((3,3))
    phimat[0,0] = 1.
    phimat[1,1] = np.cos(phik)
    phimat[1,2] = -np.sin(phik)
    phimat[2,1] = -np.sin(phik)
    phimat[2,2] = np.cos(phik)

    dqspace = phimat@dq
    dqspace = dqspace.flatten()
    qnew = np.zeros((12))

    qchassis = q[:3]

    qnew[:3] = qchassis+dqspace
    qnew[3:8] = armnew
    qnew[8:] = wheelnew

    return qnew


def TestJointLimits(thetalist, Jarm):
    """
    Check joints for potential collision zones and implements collision avoidance by 
    zeroing out the Jacobian columns approaching the collision zone.
    :param thetalist: joint angles of the robot arm (rad/s)
    :param Jarm: the current Jacobian of the robot arm
    :returns: new Jacobian of the robot arm
    """
    # Set Jacobian columns to 0 for joint arm angles if approaching collision zone
    for i in range(1, 4):
        if thetalist[i]>-0.25:
            Jarm[:,i] = np.zeros((6))
            pass

    return Jarm




def FeedbackControl(X, Xd, Xdnext,Kp,Ki, dt, q, integral_error):
    """
    Control law for the robot end effector. It can be feedorward-proportional or feedforward-proportional-integral
    :param X: The current configuration of the robot end effector
    :param Xd: The desired end-effector configuration
    :param Xdnext: The next timestep desired end-effector configuration
    :param Kp: Proportional gain of controller
    :param Ki: Integral gain of controller
    :param dt: timestep of simulation
    :param q: current state of the robot
    :param  integral error: integral error of robot
    :returns :joint command inputs, updated integral error, current error twist
    """
    
    l = 0.47/2
    w = 0.3/2
    r = 0.0475
    thetalist = q[3:8]

    #Compute pseudoinverse of H(0)
    F = np.zeros((3,4))
    F[0,0] = 0.25*r*(-1)/(l+w)
    F[0,1] = 0.25*r*1/(l+w)
    F[0,2] = 0.25*r*1/(l+w)
    F[0,3] = 0.25*r*(-1)/(l+w)
    F[1,0] = 0.25*r*1
    F[1,1] = 0.25*r*1
    F[1,2] = 0.25*r*1
    F[1,3] = 0.25*r*1
    F[2,0] = 0.25*r*(-1)
    F[2,1] = 0.25*r*1
    F[2,2] = 0.25*r*(-1)
    F[2,3] = 0.25*r*1
    F = np.vstack((np.zeros((2,4)),F ))
    F = np.vstack((F,np.zeros((1,4))))
    #Construct Jacobian
    Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                    [0, -1, 0,   -0.5076,   0,   0],
                    [0, -1, 0,   -0.3526,   0,   0],
                    [0, -1, 0, -0.2176, 0, 0],
                    [0,0,1,0,0,0]]).T
    M0e = mr.RpToTrans(np.eye(3), np.array([0.033,0,0.6546]))
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    Tb0 = mr.RpToTrans(np.eye(3), np.array([0.1662,0,0.0026]))    
   
    temp = mr.TransInv(T0e)@mr.TransInv(Tb0)
    Jb = mr.Adjoint(temp)@F
    Jarm = mr.JacobianBody(Blist, thetalist)
    Jtot = np.hstack((Jb,Jarm))

    Xerr_se3 = mr.MatrixLog6(mr.TransInv(X)@Xd)
    Xerr = np.around(mr.se3ToVec(Xerr_se3),6)
    integral_error+=Xerr*dt
    Vd_se3 = (1/dt)*mr.MatrixLog6(mr.TransInv(Xd)@Xdnext)
    Vd = mr.se3ToVec(Vd_se3)
    temp2 = np.dot(mr.TransInv(X),Xd)
    adj2 = mr.Adjoint(temp2)
    #Compute control effort
    V_computed = np.dot(adj2,Vd) + Kp@Xerr + Ki@integral_error

    #Convert to commanded inputs, check for singularity edgecases
    u_thetadot = np.linalg.pinv(Jtot,rcond=1e-3)@V_computed

    thetalist_temp = thetalist+u_thetadot[4:]*dt

    #Check for potential collisions
    Jarm = TestJointLimits(thetalist_temp, Jarm)
    Jtot = np.hstack((Jb,Jarm))
    u_thetadot = np.linalg.pinv(Jtot)@V_computed

    return u_thetadot, integral_error, Xerr

def Chassis_Trans(q):
    """Computes the chassis transform configuration relative to the space frame
    :param q: current configuration of the robot in space frame
    : returns: transform of robot relative to space frame
    """
    R = np.zeros((3,3))
    R[0,0] = np.cos(q[0])
    R[0,1] = -np.sin(q[0])
    R[1,0] = np.sin(q[0])
    R[1,1] = np.cos(q[0])
    R[2,2] = 1.
    p = np.hstack((q[1:],0.096))

    return mr.RpToTrans(R, p)


def main():
    dt = 0.01
    eintegral = 0.
    q = np.array([0.31,+0.,0,0,-0.6,-0.78,-0.78,0])
    Kp = 15*np.eye(6)#best
    # Kp = 5*np.eye(6) #overshoot
    # Kp = 40*np.eye(6) #newtask

    Ki = np.zeros((6,6))# best
    # Ki = 70*np.eye(6) # overshoot
    # Ki = np.zeros((6,6)) #newtask


    thetalist = q[3:]
    Tse_init = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])

    Tsc_init = np.array([[1,0,0,1],[0,1,0,0 ],[0,0,1,0],[0,0,0,1]]) #best and overshoot
    Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) #best and overshoot
    
    # Tsc_init = np.array([[1,0,0,1.5],[0,1,0,-0.4],[0,0,1,0],[0,0,0,1]])#new task
    # Tsc_final = np.array([[1,0,0,0.2],[0,1,0,0.1],[0,0,1,0.025],[0,0,0,1]])#new task


    Tce_grasp = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0.025],[0,0,0,1]])
    Tce_standoff = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0.15],[0,0,0,1]])
    k = 1 #you have to change time multiplier in Coppeliasim if you change k

    traj = TrajectoryGenerator(Tse_init,Tsc_init,Tsc_final,Tce_grasp, Tce_standoff, k)

    Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                    [0, -1, 0,   -0.5076,   0,   0],
                    [0, -1, 0,   -0.3526,   0,   0],
                    [0, -1, 0, -0.2176, 0, 0],
                    [0,0,1,0,0,0]]).T
    M0e = mr.RpToTrans(np.eye(3), np.array([0.033,0,0.6546]))
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    Tb0 = mr.RpToTrans(np.eye(3), np.array([0.1662,0,0.0026])) 
    Tsb = Chassis_Trans(q[:3])

    print('Starting config: ', Tsb@Tb0@T0e)
    np.savetxt("reference.csv", np.around(traj,3), delimiter=",") 
    state = np.hstack((q,np.zeros(4)))
    thetalist = state[3:8]
    hyperstate = np.hstack((state,traj[0,-1]))
    Xerr = np.zeros((1,6))

    print("Generating animation csv file.")
    for i in range(0,traj.shape[0]-1 ):

        thetalist = state[3:8]
        Tsb = Chassis_Trans(state[:3])
        T0e = mr.FKinBody(M0e, Blist, thetalist) 
        uthetadot, eintegral, Xerr_temp = FeedbackControl(Tsb@Tb0@T0e, rowtoMatrix(traj[i,:]), rowtoMatrix(traj[i+1,:]), Kp, Ki, dt, state, eintegral)
        state = NextState(state,uthetadot,dt, 100)
        hyperstate = np.vstack((hyperstate,np.hstack((state,traj[i,-1]))))
        Xerr = np.vstack((Xerr, Xerr_temp))
    
    print("Writing error plot data.")
    lwidth = 1
    plt.plot(Xerr[1:,0],label="omega x (angular)",linewidth=lwidth)
    plt.plot(Xerr[1:,1],label="omega y(angular)",linewidth=lwidth)
    plt.plot(Xerr[1:,2], label="omega z (angular)",linewidth=lwidth)
    plt.plot(Xerr[1:,3],label="vx (linear)",linewidth=lwidth)
    plt.plot(Xerr[1:,4],label="vy (linear)",linewidth=lwidth)
    plt.plot(Xerr[1:,5],label="vz (linear)",linewidth=lwidth)
    plt.xlabel("Time (s)")
    plt.ylabel("Error Twist (m/s, linear), (rad/s, angular))")
    plt.title("Error Twist vs. time")
    plt.legend()
    plt.show()


    np.savetxt("xerr.csv", np.around(Xerr,3), delimiter=",") 
    np.savetxt("Kennedy_Ian_capstone.csv", np.around(hyperstate,3), delimiter=",")
    print("Done") 
    
if __name__ == '__main__':
    main()
