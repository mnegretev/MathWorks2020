#!/usr/bin/env python
import rospy
import sys
import math
import datetime
import rospkg
from std_msgs.msg import Float64MultiArray

SAMPLING_FREQ = 250

def callback_current_pose(msg):
    global current_pose
    current_pose = msg.data

def main():
    print("INITIALIZING NODE FOR GENERATING PERSISTENT SIGNALS...")
    rospy.init_node("persistent_signals");
    rospy.Subscriber("/hardware/la_current_pose", Float64MultiArray, callback_current_pose)
    pub_torques = rospy.Publisher("/hardware/la_torque", Float64MultiArray, queue_size=10);
    loop = rospy.Rate(SAMPLING_FREQ)

    global current_pose
    current_pose = [0,0,0,0,0,0,0]
    amplitudes   = [2,0,0,0,0,0,0]
    frequencies  = [0.3,0.3,0.3,0.3,0.3,0.3,0.3]
    phases       = [0,0,0,0,0,0,0]
    offsets      = [0,0,0,0,0,0,0]

    print "Using amplitudes:  " + str(amplitudes )
    print "Using frequencies: " + str(frequencies)
    print "Using phases:      " + str(phases     )
    torques = Float64MultiArray()
    torques.data = [0,0,0,0,0,0,0]
    t = 0;

    logging_t = []
    logging_u = []
    logging_q = []
    while not rospy.is_shutdown():
        logging_t.append(t)
        logging_u.append([u for u in torques.data])
        logging_q.append(current_pose)
            
        for i in range(7):
            torques.data[i] = amplitudes[i] * math.sin(2*math.pi*frequencies[i]*t + phases[i]) + offsets[i]
        t += 1.0/SAMPLING_FREQ
        pub_torques.publish(torques)
        loop.sleep();

    print "Saving data..."
    data_str = ""
    for i in range(len(logging_t)):
        data_str += str(logging_t[i])
        for j in range(7):
            data_str += ", " + str(logging_u[i][j])
        for j in range(7):
            data_str += ", " + str(logging_q[i][j])
        data_str += "\n"
    str_now = datetime.datetime.strftime(datetime.datetime.now(), "%Y-%m-%dT%H:%M:%S");
    file_name = rospkg.RosPack().get_path("persistent_signal_gen") + "/dataset/" + str_now + ".csv"
    print "Saving to file: " + file_name
    f = open(file_name, 'w').write(data_str)
    
if __name__ == '__main__':
    main()
    
