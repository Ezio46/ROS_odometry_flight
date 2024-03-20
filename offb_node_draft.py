#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import PointCloud
import math

current_state = State()
goal_pose = PoseStamped()
current_pose = PoseStamped()
trajectory_points = []
# path_points = []

def state_cb(msg):
    global current_state
    current_state = msg


def current_pose_cb(msg):
    global current_pose
    current_pose = msg

def publish_trajectory():
    global trajectory_points
    pointcloud = PointCloud()
    pointcloud.header.stamp = rospy.Time.now()
    pointcloud.header.frame_id = "map"
    for point in trajectory_points:
        p = Point()
        p.x = point.x
        p.y = point.y
        p.z = point.z
        pointcloud.points.append(p)
    trajectory_pub.publish(pointcloud)

# def publish_path():
#     global path_points
#     path = Path()
#     path.header.stamp = rospy.Time.now()
#     path.header.frame_id = "map"
#     for point in path_points:
#         p = Poses()
#         p.x = point.x
#         p.y = point.y
#         p.z = point.z
#         path.points.append(p)
#     path_pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    rospy.loginfo("ARBITEN")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = current_pose_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    trajectory_pub = rospy.Publisher("drone_trajectory", PointCloud, queue_size=10)

    # path_pub = rospy.Publisher("drone_path", Path, queue_size=10)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 2.0

    flag1 = False
    flag2 = False

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while not rospy.is_shutdown(): # Мин. частота должна быть 10 Гц

        if (current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if (set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if (not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if (arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        # print(current_pose.pose.position.x, " ", current_pose.pose.position.y, " ", current_pose.pose.position.z)

        if math.sqrt((current_pose.pose.position.x - goal_pose.pose.position.x)**2 + (current_pose.pose.position.y - goal_pose.pose.position.y)**2 + (current_pose.pose.position.z - goal_pose.pose.position.z)**2) <= 0.1:
            goal_pose.pose.position.x = 5.0
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.position.z = 2.0
            flag1 = True

        if math.sqrt((current_pose.pose.position.x - goal_pose.pose.position.x)**2 + (current_pose.pose.position.y - goal_pose.pose.position.y)**2 + (current_pose.pose.position.z - goal_pose.pose.position.z)**2) <= 0.1:
            goal_pose.pose.position.x = 5.0
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.position.z = 0.0
            flag2 = True


        if math.sqrt((current_pose.pose.position.x - goal_pose.pose.position.x)**2 + (current_pose.pose.position.y - goal_pose.pose.position.y)**2 + (current_pose.pose.position.z - goal_pose.pose.position.z)**2) <= 0.2 and flag1 and flag2:
            break

        local_pos_pub.publish(goal_pose)
        trajectory_points.append(current_pose.pose.position)
        path_points.append(current_pose.pose.position)

    publish_trajectory()

