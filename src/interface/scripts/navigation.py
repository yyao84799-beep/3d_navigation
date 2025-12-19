#!/usr/bin/env python3
# coding:utf-8

import rospy
import roslaunch
from std_msgs.msg import Bool

launch_nav = None
nav_enabled = False


def switch_node_callback(msg: Bool):
    global nav_enabled
    nav_enabled = bool(msg.data)
    print(f"[INFO] Navigation launch switch: {nav_enabled}")


if __name__ == "__main__":
    try:
        rospy.init_node("roslaunch_navigation", anonymous=True)
        rospy.Subscriber("nav_start_stop_launch", Bool, switch_node_callback)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if nav_enabled:
                if launch_nav is None:
                    launch_file = "/home/guest/PCTplaner_ws/src/interface/launch/navigation.launch"
                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    launch_nav = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
                    launch_nav.start()
                    rospy.loginfo("[INFO] Navigation launch started.")
            else:
                if launch_nav is not None:
                    launch_nav.shutdown()
                    launch_nav = None
                    rospy.loginfo("[INFO] Navigation launch stopped.")

            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("[INFO] ROSInterruptException caught, shutting down.")
