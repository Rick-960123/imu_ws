#!/usr/bin/env python3
# HJ: 5/28/20

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
sys.path.insert(0,'/home/zhen/ZROS/ros/devel/lib/python3/dist-packages/zr_msgs')
sys.path.insert(0,'/home/zhen/.local/lib/python3.8/site-packages')
from zr_msgs.msg import motor_info
# other
import numpy as np
import math
from std_msgs.msg import Int32MultiArray
from pyquaternion import Quaternion
from geometry_msgs.msg import Quaternion as qua
from scipy.linalg import expm
from helper import to_skew

# debugging
import tf

# accelerometer
rover_stationary = False  # whether or not to do accelerometer corrections
accel_counter = 0
g_pred_sum = 0

# encoder
first_time_enc = True  # for offsets
ticks_l_prev = 0
tickr_r_prev = 0
counter = 0
pub_odom = None

def measurement_update():

    # encoder state (acts like a sensor measurement)
    global enc_state, pub_odom

    # predicted position from encoder model
    p_meas = enc_state[0][0:3]
    yaw_meas = enc_state[1][0:1]

    # encoder measurement matrix: He is 4 x 15
    He = np.hstack((np.identity(3), np.zeros((3, 12))))
    He = np.vstack(
        (He, np.concatenate((np.zeros(8), np.array([1]), np.zeros(6)))))

    # encoder noise matrix
    global enc_Re
    Re = enc_Re

    # access current predicted state
    global state
    p = state[0][0:3]
    b = state[2]
    x_a = state[3][0:3]
    x_g = state[4][0:3]

    # to pyQuaternion format
    b_q = Quaternion(array=b)
    b_inv = b_q.inverse  # body to nav
    R_body_to_nav = b_inv.rotation_matrix

    angles = tf.transformations.euler_from_quaternion(
        [b_inv[1], b_inv[2], b_inv[3], b_inv[0]], 'rzyx')

    # predicted yaw
    yaw_pred = angles[0]

    # predicted position
    p_pred = p

    # if robot stationary
    global rover_stationary
    rover_stationary = False
    # use accelerometer data if rover stationary
    if rover_stationary:

        # measurement matrix accelerometer: Ha is 2 x 15
        global g
        # Ha = np.array([[0,g,0],[-g, 0,0],[0,0,0]])
        # Ha = np.hstack((np.zeros((3,6)),Ha,R_body_to_nav,np.zeros((3,3))))

        Ha = np.array([[0, g, 0], [-g, 0, 0]])  # ERROR HERE. SIGN PROBLEM
        Ha = np.hstack(
            (np.zeros((2, 6)), Ha, R_body_to_nav[:2, :], np.zeros((2, 3))))

        # combined measurement matrices
        H = np.vstack((He, Ha))

        # get accelerometer noise matrix
        global Ra

        # combined noise matrix: R is 5 x 5
        R = np.diag(np.concatenate((np.diagonal(Re), np.diagonal(Ra))))

        # predicted gravity vector
        global g_pred

        # stack predicted 'measurement': predicted position and gravity vector
        # y_pred = np.concatenate((p_pred,np.array([yaw_pred]),g_pred))
        y_pred = np.concatenate((p_pred, np.array([yaw_pred]), g_pred[:2]))

        # known gravity acceleration
        # g_vec = np.array([0,0,g])
        g_vec = np.array([0, 0])

        # actual measurement is output position of encoder model and known gravity acceleration
        y = np.concatenate((p_meas, np.array([yaw_meas]), g_vec))

        # restart counter # should it be somewhere else
        rover_stationary = False

        print('Stationary update')

    else:

        # measurement and noise matrices depend on encoder only
        H = He
        # R = Re
        R = np.identity(4) * 0.0000001

        # predicted measurement is position and yaw
        y_pred = np.append(p_pred, yaw_pred)

        # actual measurement is output position and yaw of encoder model
        y = np.append(p_meas, yaw_meas)

    # previous covariance: 15x15
    global cov

    # compute Kalman gain
    try:
        print("R: {}".format(R))
        K_ = np.linalg.inv(R+H.dot(cov).dot(H.T))
        K = cov.dot(H.T).dot(K_)
    except:
        print('Kalman gain error')
        return

    # residual z
    z = y-y_pred

    # angles can be close to pi and -pi
    if abs(z[3]) > math.pi:
        z[3] -= np.sign(z[3])*2*math.pi

    # compute state correction
    dx = K.dot(z)

    # rotation update
    ro = dx[6:9]
    P = to_skew(ro)
    R_nav_to_body = R_body_to_nav.T
    R_nav_to_body_next = R_nav_to_body.dot(np.identity(3) - P)  # (10.67)

    # reorthogonalize matrix... required by PyQuaternion... use SVD
    U, S, Vh = np.linalg.svd(R_nav_to_body_next)
    R_nav_to_body_next = U.dot(Vh)

    # print(R_nav_to_body_next)
    # compute next quaternion
    t = np.identity(4)
    t[0:3,0:3] = R_nav_to_body_next
    b_next =tf.transformations.quaternion_from_matrix(t)
    # # update state
    if np.linalg.norm(dx[:3]) > 0.1:
        print('pos update crazy')
    # print(ro)
    # 	print(p_pred)
    # 	print(p_meas)
    # 	print()
    if np.linalg.norm(dx[3:6]) > 1:
        print('vel update crazy')

    # update state
    state[0][0:3] = state[0][0:3] + dx[:3]
    state[1][0:3] = state[1][0:3] + dx[3:6]
    state[2] = b_next  # update quaternion: [w, x, y, z]
    state[3][0:3] = state[3][0:3] + dx[9:12]  # update accel bias
    state[4][0:3] = state[4][0:3] + dx[12:]  # update gyro bias

    # update covariance
    cov = (np.identity(15)-K.dot(H)).dot(cov)

    # update encoder start state: position and orientation
    enc_state[0] = state[0]
    enc_state[2] = b_next

    global counter
    # if counter % 20 == 0:
        # print(state[0])

    counter += 1

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "123"
    odom.pose.pose.position.x = state[0][0]
    odom.pose.pose.position.y = state[0][1]
    odom.pose.pose.position.z = 0.0
    odom_quat = qua(b_next[3],b_next[0],b_next[1],b_next[2])
    odom.pose.pose.orientation = odom_quat
    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = 0.0
    pub_odom.publish(odom)


last_V_R, last_V_L, last_motorinfo_stamp = 0.0, 0.0, 0.0


def encoders_callback(data):

    # encoder ticks may start as non_zero. record offsets
    global first_time_enc, last_motorinfo_stamp, last_V_L, last_V_R

    if first_time_enc:
        last_motorinfo_stamp = data.header.stamp.to_sec()
        last_V_L = data.left_vel
        last_V_R = data.right_vel
        first_time_enc = False
        return

    V_L = data.left_vel
    V_R = data.right_vel
    dt = data.header.stamp.to_sec() - last_motorinfo_stamp

    Dl = V_L * dt
    Dr = V_R * dt
    Dc = (Dl + Dr) / 2.0

    # get current position estimate. starts at end of last measurement update
    global enc_state
    p = enc_state[0][0:3]
    x = p[0]
    y = p[1]
    z = p[2]

    # orientation
    b = enc_state[2]
    b_q = Quaternion(array=b)

    # get current yaw pitch roll. tf: Quaternions ix+jy+kz+w are represented as [x, y, z, w].
    b_inv = b_q.inverse
    angles = tf.transformations.euler_from_quaternion(
        [b_inv[1], b_inv[2], b_inv[3], b_inv[0]], 'rzyx')

    # store angles in global variables
    phi = angles[2]  # roll
    theta = angles[1]  # pitch
    psi = angles[0]  # yaw

    L = 0.5

    # kinematic model
    x = x + Dc*math.cos(psi)
    y = y + Dc*math.sin(psi)
    z = 0.0
    yaw = psi

    # store in state
    enc_state[0][0:3] = np.array([x, y, z])
    enc_state[1][0:1] = yaw

    # encoder covariance
    global enc_Re, k
    cpsi = math.cos(psi)
    spsi = math.sin(psi)
    ctheta = math.cos(theta)
    stheta = math.sin(theta)
    cphi = math.cos(phi)

    # variance = k*Dl
    U = np.diag([(k*np.abs(Dl))**2, (k*np.abs(Dr))**2])

    # with yaw
    Fu = np.array([[0.5*cpsi*ctheta, 0.5*cpsi*ctheta], [0.5*psi*ctheta, 0.5*psi*ctheta],
                  [-0.5*stheta, -0.5*stheta], [-1/L*cphi/ctheta, 1/L*cphi/ctheta]])

    # store encoder noise in global variable
    enc_Re = Fu.dot(U).dot(Fu.T)

    measurement_update()

    last_motorinfo_stamp = data.header.stamp.to_sec()


# update orientation here. use RK3
def imu_callback(data):

    # sample time
    global dt
    dt = 1/100.0

    # get gyro data
    w = np.array([data.angular_velocity.x,
                 data.angular_velocity.y, data.angular_velocity.z])

    # get accelerometer data: specific force in body frame
    f_b = np.array([data.linear_acceleration.x,
                   data.linear_acceleration.y, data.linear_acceleration.z])

    ### Update state ###

    # get current state
    global state

    # current position
    p = state[0][0:3]

    # current velocity
    v = state[1][0:3]

    # current orientation
    b = state[2]

    # current accel bias
    x_a = state[3][0:3]

    # current gyro bias
    x_g = state[4][0:3]

    # subtract out gyroscope bias. w_bn = (-w_nb)
    w = -1*(w-x_g)
    w_norm = np.linalg.norm(w)

    # subtract out accelerometer bias
    f_b = f_b - x_a

    # differential rotation: [w, x, y, z]
    db = np.concatenate(
        ([math.cos(w_norm*dt/2)], math.sin(w_norm*dt/2)*w/w_norm))

    # convert to pyquaternion format
    b_prev = Quaternion(array=b)
    db = Quaternion(array=db)

    # update orientation
    b_next = db*b_prev

    # get average quaternion by interpolation
    b_avg = Quaternion.slerp(b_prev, b_next, amount=0.5)

    # b is the nav to body transformation. we need body to nav transformation -> invert
    # for specific force (5.9 Principles of GNSS book)
    b_body_to_nav_avg = b_avg.inverse

    # rotate specific force into inertial frame
    f_i = b_body_to_nav_avg.rotate(f_b)

    # gravity vector
    global g
    g_vec = np.array([0, 0, g])

    # get acceleration in inertial frame. (acceleration of body wrt inertial frame in inertial frame)
    a_i = f_i + g_vec

    # update position (5.16 Principles of GNSS book)
    p = p + v*dt + 0.5*a_i*dt**2

    # DEBUG:
    if np.linalg.norm(v*dt + 0.5*a_i*dt**2) > 0.1:
        print("IMU update crazy")
        # print(state)
    # 	print(np.linalg.norm(v*dt + 0.5*a_i*dt**2) )
    # 	print(v)
    # 	print(a_i)
    # 	print(x_a)
    # 	print(x_g)
    # 	print(w)
    # 	print()
    # 	rospy.sleep(100)

    # update velocity
    v = v + a_i*dt

    # store in state -> this is time propagation step.
    state[0][0:3] = p
    state[1][0:3] = v
    state[2] = b_next.elements

    ### Update covariance ###

    # get rotation matrix corresponding to b_next
    b_body_to_nav = b_next.inverse
    R_body_to_nav = b_body_to_nav.rotation_matrix

    # compute F matrix: F is 15 x 15
    F = np.zeros((15, 15))

    # store relevant values in F (see writeup)
    F[:3, 3:6] = np.identity(3)
    f_i_skew = to_skew(f_i)
    F[3:6, 6:9] = -1*f_i_skew
    F[3:6, 9:12] = -1*R_body_to_nav
    F[6:9, 12:15] = R_body_to_nav
    F_gg = -1.0/1000*np.identity(3)
    F_aa = -1.0/1000*np.identity(3)
    F[9:12, 9:12] = F_aa
    F[12:15, 12:15] = F_gg

    # compute system transition matrix Phi
    Phi = expm(F*dt)

    # compute G. G is 15 x 12.
    G = np.zeros((15, 12))
    G[3:6, :3] = -R_body_to_nav
    G[6:9, 3:6] = R_body_to_nav
    G[9:12, 6:9] = np.identity(3)
    G[12:15, 9:12] = np.identity(3)

    # get noise matrix
    global Q

    # compute Qdk (discrete noise). Qdk is 15 x 15
    # Qdk = G.dot(Q).dot(G.T)*dt
    Qdk = np.identity(15)
    print("Q: {}".format(Qdk))
    # get previous covariance
    global cov

    # update covariance (15x15)
    cov = Phi.dot(cov).dot(Phi.T)+Qdk

    ### Measurement update stuff for acceleration. Helps correct roll and pitch ###

    # if 50 accelerometer readings were close enough to the gravity vector, robot is stationary
    global accel_counter, rover_stationary, g_pred_sum

    # predict gravity in navigation frame and store prediction in global variable. used in filter.
    num_g_pred = 50
    global g_pred_sum

    # check if current measurement is stationary
    # tuned. test: you should never be able to hold the imu in your hand and have an update.
    if np.abs(np.linalg.norm(f_b) - g) < 0.03:
        accel_counter += 1
        g_pred_sum += R_body_to_nav.dot(-1*f_b)
    else:
        accel_counter = 0
        g_pred_sum = 0
        rover_stationary = False

    # if 50 consecutive stationary, use accel_data
    if accel_counter == num_g_pred:
        global g_pred
        # averaging
        g_pred = g_pred_sum/num_g_pred
        rover_stationary = True
        accel_counter = 0
        g_pred_sum = 0

    ### Visualization ###

    # for visualization: publish tf
    if True:
        b_vis = b_body_to_nav

        # useful for visualization
        br = tf.TransformBroadcaster()

        # send world to IMU transformation
        br.sendTransform((p[0], p[1], p[2]),
                         (b_vis[1], b_vis[2], b_vis[3], b_vis[0]),
                         rospy.Time.now(),
                         "IMU",
                         "ENU")  # ENU to quat


def initialize():

    # print("aided_nav: waiting for initialize_ahrs service")
    # # wait for service to become active
    # rospy.wait_for_service('initalize_ahrs')

    try:
        # create service method
        # initalize_ahrs = rospy.ServiceProxy('initalize_ahrs', initRequest)

        # request the service
        print("aided_nav: Initializing orientation and gyro bias.")
        # resp = initalize_ahrs()
        print("aided_nav: Received initalization data.")

        # access service response
        # b = resp.init_orientation
        # gyro_bias = resp.gyro_bias

        # initialize state: [p, v, b, x_a, x_g] = [position, velocity, quaternion, accel bias, gyro bias],  size 16
        global state
        state = np.array([np.zeros(4), np.zeros(4), np.array([1,0,0,0]), np.zeros(
            4), np.zeros(4)])

        # initialize noise terms
        global dt
        hz = rospy.get_param("imu_hz", 100)
        dt = 1.0/hz
        T = 1000.0/hz  # number of measurements over rate of IMU
        # Gyro (rate) random walk
        sigma_xg = rospy.get_param('sigma_xg', 1)
        # rad/s/rt_Hz, Gyro white noise
        sigma_nug = rospy.get_param('sigma_nug', 1)
        # Accel (rate) random walk m/s3 1/sqrt(Hz)
        sigma_xa = rospy.get_param('sigma_xa', 1)
        sigma_nua = rospy.get_param(
            'sigma_nua', 1)  # accel white noise

        # compute noise matrix for IMU, Q. Q is 12 x 12.
        global Q
        Q = np.zeros((12, 12))
        Q[:3, :3] = sigma_nua**2*np.identity(3)
        Q[3:6, 3:6] = sigma_nug**2*np.identity(3)
        Q[6:9, 6:9] = sigma_xa**2*np.identity(3)
        Q[9:12, 9:12] = sigma_xg**2*np.identity(3)

        # accelerometer noise matrix, used in measurment update. Ra is 3x3
        global Ra
        # Ra = np.identity(3)*sigma_nua**2
        Ra = np.identity(2)*sigma_nua**2

        # gravity vector
        global g
        # TODO: put in launch file
        g = 9.81  # gravity m/s/s

        # initialize covariance: cov is 15 x 15
        global cov
        cov = np.zeros((15, 15))
        cov[6:9, 6:9] = np.diag([sigma_nua/g, sigma_nua/g, 0])**2/T  # Ppp
        cov[9:12, 9:12] = np.zeros((3, 3))  # Paa
        cov[12:, 12:] = np.identity(3)*sigma_nug**2/T  # Pgg

        # initialize encoder model state [p,psi,b] = [position, yaw, orientation]
        # encoder acts like a sensor initialized at end of every measurement update
        global enc_state
        enc_state = np.array([np.zeros(4), np.zeros(4), np.array([1,0,0,0])])

        # encoder noise model
        global k
        k = 0.05  # % slip

    except rospy.ServiceException as e:
        print ("Service call failed:{}".format(e))


def main():

    global pub_odom
    # initialize node
    rospy.init_node('aided_nav', anonymous=True)

    # initialize parameters
    initialize()

    # subscribe to encoders
    rospy.Subscriber("/motor_info", motor_info,
                     encoders_callback, queue_size=1)

    # subscribe to IMU
    rospy.Subscriber("/imu_data", Imu, imu_callback, queue_size=1)

    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
