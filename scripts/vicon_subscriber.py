#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np

N_bots=4
BotNumber=0  #This will be set correctly (0,1,2...N_bots) so that the bot know which location data is its own. 

bot_loc=np.empty((2,N_bots));

def callbackVICON(data, arg):
    global bot_loc
    global BotNumber
    bot_index=arg
    #Temporary shiz
    bot_loc[:,bot_index]=(data.transform.translation.x,data.transform.translation.y);
    if arg==BotNumber:
        print bot_loc
    
def vicon_listen():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vicon_listen', anonymous=True)

    rospy.Subscriber("/vicon/Ani_robot_1/Ani_robot_1", TransformStamped, callbackVICON,1)
    rospy.Subscriber("/vicon/fixed_1/fixed_1", TransformStamped, callbackVICON,2)
    rospy.Subscriber("/vicon/lambo/lambo", TransformStamped, callbackVICON,3)
    rospy.Subscriber("/vicon/maulik_bot_2/maulik_bot_2", TransformStamped, callbackVICON,0)

    #rospy.Subscriber("/vicon/vijeth_1/vijeth_1", TransformStamped, callbackVICON,1)
    #rospy.Subscriber("/vicon/vijeth_2/vijeth_2", TransformStamped, callbackVICON,2)
    #rospy.Subscriber("/vicon/vijeth_3/vijeth_3", TransformStamped, callbackVICON,3)
    #rospy.Subscriber("/vicon/vijeth_0/vijeth_0", TransformStamped, callbackVICON,0)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    vicon_listen()