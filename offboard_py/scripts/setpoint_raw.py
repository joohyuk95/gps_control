#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from sensor_msgs.msg import NavSatFix

current_state = State()
current_pos = NavSatFix()

def state_cb(msg):
    global current_state
    current_state = msg

def pos_cb(msg):
    global current_pos
    current_pos = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pos_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, callback = pos_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    global_pos_pub = rospy.Publisher("mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=1)
    
    
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


    raw_vel = GlobalPositionTarget()
    raw_vel.coordinate_frame = 5
    # raw_vel.type_mask = 0b110111111000 
    raw_vel.type_mask = 0b100111111111
    raw_vel.latitude = 47.3978656
    raw_vel.longitude = 8.5456460
    raw_vel.altitude = 490
    # raw_vel.velocity.x = 1.0
    # raw_vel.velocity.y = 1.0
    # raw_vel.velocity.z = 1.0
    
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