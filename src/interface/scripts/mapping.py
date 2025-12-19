#!/usr/bin/env python3
# coding:utf-8

import rospy
import roslaunch
from std_msgs.msg import Bool

launch1 = None
launch2 = None
file_to_launch = False


def switch_node_callback(msg):
    """话题回调：更新建图开关"""
    global file_to_launch
    file_to_launch = msg.data
    print(f"[INFO] Build launch switch: {file_to_launch}")


if __name__ == "__main__":
    try:
        rospy.init_node("roslaunch_build_map", anonymous=True)
        rospy.Subscriber("build_start_stop_launch", Bool, switch_node_callback)

        rate = rospy.Rate(1)  # 每秒检查一次开关

        while not rospy.is_shutdown():
            if file_to_launch:
                if launch2 is None:
                    # 启动建图 launch
                    launch_file2 = "/home/guest/PCTplaner_ws/src/FAST_LIO/launch/mapping_mid360.launch"
                    uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid2)
                    launch2 = roslaunch.parent.ROSLaunchParent(uuid2, [launch_file2])
                    launch2.start()
                    rospy.loginfo("[INFO] Build launch started.")
            else:
                if launch2 is not None:
                    # 停止建图 launch
                    launch2.shutdown()
                    launch2 = None
                    rospy.loginfo("[INFO] Build launch stopped.")

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("[INFO] ROSInterruptException caught, shutting down.")
