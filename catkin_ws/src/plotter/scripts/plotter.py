#!/usr/bin/env python
import rospy
import matplotlib.pyplot
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

SAMPLING_FREQUENCY = 250

def callback_estimated_speed(msg):
    global estimated_speed
    estimated_speed = msg.data
    
def callback_torque         (msg):
    global torque
    torque  = msg.data
    
def callback_estimated_mass (msg):
    global estimated_mass
    estimated_mass = msg.data
    
def main():
    print "Intializing plotter node ..."
    rospy.init_node("plotter")
    rospy.Subscriber("/hardware/la_estimated_speed", Float64MultiArray, callback_estimated_speed)
    rospy.Subscriber("/hardware/la_torque"         , Float64MultiArray, callback_torque         )
    rospy.Subscriber("/estimated_mass"             , Float64          , callback_estimated_mass )
    loop = rospy.Rate(SAMPLING_FREQUENCY);

    global estimated_speed, torque, estimated_mass
    estimated_speed = [0,0,0,0,0,0,0]
    torque          = [0,0,0,0,0,0,0]
    estimated_mass  = 0
    speeds  = []
    torques = []
    plt_masses  = []
    plt_time = []
    t = 0
    while not rospy.is_shutdown():
        speeds.append(estimated_speed)
        torques.append(torque)
        plt_masses.append(estimated_mass)
        t += 1.0/SAMPLING_FREQUENCY
        plt_time.append(t)
        loop.sleep()
        
    print "Arranging data..."
    plt_speeds  = []
    plt_torques = []
    for i in range(len(speeds[0])):
        s = [speeds [j][i] for j in range(len(speeds ))]
        t = [torques[j][i] for j in range(len(torques))]
        plt_speeds.append(s)
        plt_torques.append(t)
    matplotlib.pyplot.title("Estimated speeds")
    for i in range(len(plt_speeds)):
        matplotlib.pyplot.plot(plt_time, plt_speeds[i])
        
    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("Torques")
    for i in range(len(plt_torques)):
        matplotlib.pyplot.plot(plt_time, plt_torques[i])
        
    matplotlib.pyplot.figure()
    matplotlib.pyplot.title("Estimated mass")
    matplotlib.pyplot.plot(plt_time, plt_masses)
    matplotlib.pyplot.show()

if __name__ == '__main__':
    main()
