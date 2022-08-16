#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State, PositionTarget, HomePosition
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from math import atan2
 
current_state = State()
current_pos = NavSatFix()
home_pos = HomePosition()

def state_cb(msg):
    global current_state
    current_state = msg

def pos_cb(msg):
    global current_pos
    current_pos = msg

def home_cb(msg):
    global home_pos
    home_pos = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pos_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, callback = pos_cb)
    home_sub = rospy.Subscriber("mavros/home_position/home", HomePosition, callback = home_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    global_pos_pub = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)
    
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2


    raw_vel = GeoPoseStamped()
    raw_vel.pose.position.latitude = target_lat = 47.3973543
    raw_vel.pose.position.longitude = target_lon = 8.5469896
    raw_vel.pose.position.altitude = 490
    
    home_lat = home_pos.geo.latitude
    home_lon = home_pos.geo.longitude

    delta_lat = target_lat - home_lat
    delta_lon = target_lon - home_lon

    yaw_angle = atan2(delta_lat, delta_lon)

    q = quaternion_from_euler(0, 0, yaw_angle)
    raw_vel.pose.orientation.x = q[0]
    raw_vel.pose.orientation.y = q[1]
    raw_vel.pose.orientation.z = q[2]
    raw_vel.pose.orientation.w = q[3]

    margin = 0.2
    is_takeoff = True

    # Send a few setpoints before starting
    for i in range(50):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        # print(current_pos.altitude)

        if is_takeoff and (abs(537.3 - current_pos.altitude) > margin):
            local_pos_pub.publish(pose)
        else:
            is_takeoff = False
            global_pos_pub.publish(raw_vel)
        rate.sleep()