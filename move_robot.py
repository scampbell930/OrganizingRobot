from classifier import stream_vision_sensor, verify
import math
from random import random
from time import sleep
import numpy as np
import sim as vrep # access all the VREP elements
from scipy.spatial.transform import Rotation as R

def move_robot(clientID, coordinates):
    err_code,fr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fr", vrep.simx_opmode_blocking)
    err_code,fl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fl", vrep.simx_opmode_blocking)
    err_code,rr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rr", vrep.simx_opmode_blocking)
    err_code,rl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rl", vrep.simx_opmode_blocking)
    err_code,robot_handle = vrep.simxGetObjectHandle(clientID, "/youBot", vrep.simx_opmode_blocking)
    err_code,vision_handle = vrep.simxGetObjectHandle(clientID, "/youBot/Vision_sensor", vrep.simx_opmode_blocking)
    err_code,dummy_handle = vrep.simxGetObjectHandle(clientID, "/test_cube", vrep.simx_opmode_blocking)
 
    # Move to each point within the coordinate list
    for coord in coordinates:
        # Check vision sensor for object to detect
        detection = stream_vision_sensor("Vision_sensor", clientID)

        if detection:
            # Detected an object
            if detection[0]:
                # Stop moving
                err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0, vrep.simx_opmode_streaming)

                if verify(clientID, "Vision_sensor", [fr_wheel_handle, rr_wheel_handle, fl_wheel_handle, rl_wheel_handle],
                          detection):
                    return [True, detection[1]]

        err_code = vrep.simxSetObjectPosition(clientID, dummy_handle, -1, [coord.x, coord.y, 0.2],
                                               vrep.simx_opmode_blocking)
        errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
        # Create vector from robot postion to dummy position
        vector_from_points = [(coord.x - robot_position[0]), (coord.y - robot_position[1])]
        # Calculate angle relative to absolute x-axis
        vector_angle = math.atan2(vector_from_points[1], vector_from_points[0])

        errcode, vision_position = vrep.simxGetObjectPosition(clientID, vision_handle, -1, vrep.simx_opmode_blocking)

        robot_vector_from_points = [(vision_position[0] - robot_position[0]), (vision_position[1] - robot_position[1])]

        orientation_angle = math.atan2(robot_vector_from_points[1], robot_vector_from_points[0])
        errcode, robot_orientation = vrep.simxGetObjectOrientation(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
        # Calculate difference between robot orientation and vector angle to determine which motor should turn

        if (not (round(vector_angle, 1) - 0.1 <= round(orientation_angle, 1) <= round(vector_angle, 1) + 0.1)):
            if (vector_angle - orientation_angle >= 0):
                err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, -0.5, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, -0.5, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0.5, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0.5, vrep.simx_opmode_streaming)
            else:
                err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0.5, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0.5, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, -0.5, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, -0.5, vrep.simx_opmode_streaming)

        sleep(0.05)

        # Rotate until robot is orientated towards point
        while True:

            errcode, robot_orientation = vrep.simxGetObjectOrientation(clientID, robot_handle, dummy_handle,
                                                                        vrep.simx_opmode_blocking)

            errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
            errcode, vision_position = vrep.simxGetObjectPosition(clientID, vision_handle, -1, vrep.simx_opmode_blocking)
            current_vector_from_points = [(vision_position[0] - robot_position[0]),
                                           (vision_position[1] - robot_position[1])]
            angle = math.atan2(current_vector_from_points[1], current_vector_from_points[0])

            if round(vector_angle, 1) - 0.1 <= round(angle, 1) <= round(vector_angle, 1) + 0.1:
                # sleep(0.5)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0, vrep.simx_opmode_streaming)
                sleep(0.05)
                break

        # Move towards point until robot location == point location
        while True:
            errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
            # errcode, dummy_position = vrep.simxGetObjectPosition(clientID, dummy_handle,-1,vrep.simx_opmode_blocking)

            if (round(coord.x, 1) - 0.15 <= round(robot_position[0], 1) <= round(coord.x, 1) + 0.15 and
                    round(coord.y, 1) - 0.15 <= round(robot_position[1], 1) <= round(coord.y, 1) + 0.15):

                err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0, vrep.simx_opmode_streaming)
                sleep(0.1)
                break
            else:
                err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, -1, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, -1, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, -1, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, -1, vrep.simx_opmode_streaming)
                sleep(0.1)


def move_to_pickup(clientID, detection):

    err_code,fr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fr", vrep.simx_opmode_blocking)
    err_code,fl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_fl", vrep.simx_opmode_blocking)
    err_code,rr_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rr", vrep.simx_opmode_blocking)
    err_code,rl_wheel_handle = vrep.simxGetObjectHandle(clientID, "/youBot/rollingJoint_rl", vrep.simx_opmode_blocking)
    err_code,robot_handle = vrep.simxGetObjectHandle(clientID, "/youBot", vrep.simx_opmode_blocking)
    err_code,vision_handle = vrep.simxGetObjectHandle(clientID, "/youBot/Vision_sensor", vrep.simx_opmode_blocking)
    err_code,dummy_handle = vrep.simxGetObjectHandle(clientID, "/Cuboid[4]", vrep.simx_opmode_blocking)
    sleep(0.5)

    errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
    err, euler = vrep.simxGetObjectOrientation(clientID, robot_handle, -1, vrep.simx_opmode_streaming)
    sleep(0.1)

    if detection[1] == "blue":
        err, object = vrep.simxGetObjectHandle(clientID, "blue_cube", vrep.simx_opmode_blocking)
        errcode, object_position = vrep.simxGetObjectPosition(clientID, object, -1, vrep.simx_opmode_blocking)
        sleep(0.5)

        errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
        # Create vector from robot postion to dummy position
        vector_from_points = [(object_position[0] - robot_position[0]), (object_position[1] - robot_position[1])]
        # Calculate angle relative to absolute x-axis
        vector_angle = math.atan2(vector_from_points[1], vector_from_points[0])

        errcode, vision_position = vrep.simxGetObjectPosition(clientID, vision_handle, -1, vrep.simx_opmode_blocking)

        robot_vector_from_points = [(vision_position[0] - robot_position[0]), (vision_position[1] - robot_position[1])]

        orientation_angle = math.atan2(robot_vector_from_points[1], robot_vector_from_points[0])
    else:
        err, object = vrep.simxGetObjectHandle(clientID, "red_cylinder", vrep.simx_opmode_blocking)
        errcode, object_position = vrep.simxGetObjectPosition(clientID, object, -1, vrep.simx_opmode_blocking)
        sleep(0.5)

        errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
        # Create vector from robot postion to dummy position
        vector_from_points = [(object_position[0] - robot_position[0]), (object_position[1] - robot_position[1])]
        # Calculate angle relative to absolute x-axis
        vector_angle = math.atan2(vector_from_points[1], vector_from_points[0])

        errcode, vision_position = vrep.simxGetObjectPosition(clientID, vision_handle, -1, vrep.simx_opmode_blocking)

        robot_vector_from_points = [(vision_position[0] - robot_position[0]), (vision_position[1] - robot_position[1])]

        orientation_angle = math.atan2(robot_vector_from_points[1], robot_vector_from_points[0])

    # Orient to object
    while not (round(vector_angle, 3) - 0.01 <= round(orientation_angle, 3) <= round(vector_angle, 3) + 0.01):
        if (vector_angle - orientation_angle >= 0):
            err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, -0.1, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, -0.1, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0.1, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0.1, vrep.simx_opmode_streaming)
        else:
            err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0.1, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0.1, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, -0.1, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, -0.1, vrep.simx_opmode_streaming)

        sleep(0.05)
        err, euler = vrep.simxGetObjectOrientation(clientID, robot_handle, -1, vrep.simx_opmode_buffer)
        sleep(0.05)
        rob_deg = R.from_rotvec(euler)
        euler = rob_deg.as_euler('zxy', degrees=True)

        errcode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_blocking)
        # Create vector from robot postion to dummy position
        vector_from_points = [(object_position[0] - robot_position[0]), (object_position[1] - robot_position[1])]
        # Calculate angle relative to absolute x-axis
        vector_angle = math.atan2(vector_from_points[1], vector_from_points[0])

        errcode, vision_position = vrep.simxGetObjectPosition(clientID, vision_handle, -1,
                                                              vrep.simx_opmode_blocking)

        robot_vector_from_points = [(vision_position[0] - robot_position[0]),
                                    (vision_position[1] - robot_position[1])]

        orientation_angle = math.atan2(robot_vector_from_points[1], robot_vector_from_points[0])

    err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0,
                                               vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0,
                                               vrep.simx_opmode_streaming)

    # Drive to object
    while not (round(object_position[0], 2) - 0.3 <= round(vision_position[0], 2) <= round(object_position[0], 2) + 0.3 and
            round(object_position[1], 2) - 0.3 <= round(vision_position[1], 2) <= round(object_position[1], 2) + 0.3):
        errcode, vision_position = vrep.simxGetObjectPosition(clientID, vision_handle, -1,
                                                              vrep.simx_opmode_blocking)

        err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, -0.3, vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, -0.3, vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, -0.3,
                                                   vrep.simx_opmode_streaming)
        err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, -0.3,
                                                   vrep.simx_opmode_streaming)

    err_code = vrep.simxSetJointTargetVelocity(clientID, fr_wheel_handle, 0, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, rr_wheel_handle, 0, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, fl_wheel_handle, 0,
                                               vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, rl_wheel_handle, 0,
                                               vrep.simx_opmode_streaming)