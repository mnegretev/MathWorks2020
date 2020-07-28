#!/usr/bin/env python
import rospy
import matplotlib.pyplot
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

SAMPLING_FREQUENCY = 250

def callback_current_pose        (msg):
    global current_pose
    current_pose = msg.data
    
def callback_ekf_estimated_speeds(msg):
    global ekf_estimated_speeds
    ekf_estimated_speeds = msg.data
    
def callback_ekf_estimated_q     (msg):
    global ekf_estimated_q
    ekf_estimated_q = msg.data
    
def callback_smo_estimated_speeds(msg):
    global smo_estimated_speeds
    smo_estimated_speeds = msg.data
    
def callback_torque              (msg):
    global torque
    torque = msg.data
    
def callback_estimated_mass      (msg):
    global estimated_mass
    estimated_mass = msg.data
    
def main():
    print "Intializing plotter node ..."
    rospy.init_node("plotter")
    rospy.Subscriber("/hardware/la_current_pose", Float64MultiArray, callback_current_pose        )
    rospy.Subscriber("/ekf/estimated_speeds"    , Float64MultiArray, callback_ekf_estimated_speeds)
    rospy.Subscriber("/ekf/estimated_q"         , Float64MultiArray, callback_ekf_estimated_q     )
    rospy.Subscriber("/smo/estimated_speeds"    , Float64MultiArray, callback_smo_estimated_speeds)
    rospy.Subscriber("/hardware/la_torque"      , Float64MultiArray, callback_torque              )
    rospy.Subscriber("/estimated_mass"          , Float64          , callback_estimated_mass      )
    loop = rospy.Rate(SAMPLING_FREQUENCY);

    global current_pose, ekf_estimated_speeds, ekf_estimated_q, smo_estimated_speeds, torque, estimated_mass
    current_pose         = [0,0,0,0,0,0,0]
    ekf_estimated_speeds = [0,0,0,0,0,0,0]
    ekf_estimated_q      = [0,0,0,0,0,0,0]
    smo_estimated_speeds = [0,0,0,0,0,0,0]
    torque               = [0,0,0,0,0,0,0]
    estimated_mass       = 0              

    data_current_pose         = []
    data_ekf_estimated_speeds = []
    data_ekf_estimated_q      = []
    data_smo_estimated_speeds = []
    data_torque               = []
    data_estimated_mass       = []

    plt_current_pose         = []              
    plt_ekf_estimated_speeds = []              
    plt_ekf_estimated_q      = []              
    plt_smo_estimated_speeds = []              
    plt_torque               = []              
    plt_estimated_mass       = []
    plt_time                 = []
    
    t = 0
    while not rospy.is_shutdown():
        data_current_pose        .append(current_pose        )
        data_ekf_estimated_speeds.append(ekf_estimated_speeds)
        data_ekf_estimated_q     .append(ekf_estimated_q     )
        data_smo_estimated_speeds.append(smo_estimated_speeds)
        data_torque              .append(torque              )
        data_estimated_mass      .append(estimated_mass      )

        t += 1.0/SAMPLING_FREQUENCY
        plt_time.append(t)
        loop.sleep()
        
    print "Arranging data..."
    for i in range(len(data_current_pose[0])):
        p = [data_current_pose        [j][i] for j in range(len(data_current_pose        ))]
        s = [data_ekf_estimated_speeds[j][i] for j in range(len(data_ekf_estimated_speeds))]
        q = [data_ekf_estimated_q     [j][i] for j in range(len(data_ekf_estimated_q     ))]
        o = [data_smo_estimated_speeds[j][i] for j in range(len(data_smo_estimated_speeds))]
        t = [data_torque              [j][i] for j in range(len(data_torque              ))]
        plt_current_pose        .append(p)
        plt_ekf_estimated_speeds.append(s)
        plt_ekf_estimated_q     .append(q)
        plt_smo_estimated_speeds.append(o)
        plt_torque              .append(t)
        
    matplotlib.pyplot.title("Measured joint positions")
    for i in range(len(plt_current_pose)):
        matplotlib.pyplot.plot(plt_time, plt_current_pose[i])
        
    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("EKF Estimated speeds")
    for i in range(len(plt_ekf_estimated_speeds)):
        matplotlib.pyplot.plot(plt_time, plt_ekf_estimated_speeds[i])

    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("EKF Estimated positions")
    for i in range(len(plt_ekf_estimated_q)):
        matplotlib.pyplot.plot(plt_time, plt_ekf_estimated_q[i])

    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("SMO Estimated speeds")
    for i in range(len(plt_smo_estimated_speeds)):
        matplotlib.pyplot.plot(plt_time, plt_smo_estimated_speeds[i])

    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("Control Torque")
    for i in range(len(plt_torque)):
        matplotlib.pyplot.plot(plt_time, plt_torque[i])
        
    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("Estimated mass")
    matplotlib.pyplot.plot(plt_time, data_estimated_mass)
    matplotlib.pyplot.show()

if __name__ == '__main__':
    main()
