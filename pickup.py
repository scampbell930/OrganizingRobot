import sim as vrep
from time import sleep


def pickup_object(clientID):
    err_code, first_joint = vrep.simxGetObjectHandle(clientID, "/youBot/youBotArmJoint0/youBotArmJoint1",
                                                     vrep.simx_opmode_blocking)
    err_code, second_joint = vrep.simxGetObjectHandle(clientID,
                                                      "/youBot/youBotArmJoint0/youBotArmJoint1/youBotArmJoint2",
                                                      vrep.simx_opmode_blocking)
    err_code, third_joint = vrep.simxGetObjectHandle(clientID,
                "/youBot/youBotArmJoint0/youBotArmJoint1/youBotArmJoint2/youBotArmJoint3", vrep.simx_opmode_blocking)

    err_code, gripper1 = vrep.simxGetObjectHandle(clientID,
            "/youBot/youBotArmJoint0/youBotArmJoint1/youBotArmJoint2/youBotArmJoint3/youBotArmJoint4/youBotGripperJoint1",
                                                      vrep.simx_opmode_blocking)
    err_code, gripper2 = vrep.simxGetObjectHandle(clientID,
            "/youBot/youBotArmJoint0/youBotArmJoint1/youBotArmJoint2/youBotArmJoint3/youBotArmJoint4/youBotGripperJoint1/youBotGripperJoint2",
                                                  vrep.simx_opmode_blocking)

    # Move to object
    err = vrep.simxSetJointTargetVelocity(clientID, gripper2, -0.1, vrep.simx_opmode_oneshot)
    sleep(0.5)
    err = vrep.simxSetJointTargetVelocity(clientID, gripper2, 0, vrep.simx_opmode_oneshot)
    err = vrep.simxSetJointTargetPosition(clientID, first_joint, -1.5, vrep.simx_opmode_oneshot)
    sleep(5)
    err = vrep.simxSetJointTargetPosition(clientID, first_joint, -1.5, vrep.simx_opmode_oneshot)
    err = vrep.simxSetJointTargetPosition(clientID, second_joint, -0.75, vrep.simx_opmode_oneshot)
    err = vrep.simxSetJointTargetPosition(clientID, third_joint, -0.1, vrep.simx_opmode_oneshot)
    print(err)
    sleep(5)

    err = vrep.simxSetJointTargetVelocity(clientID, gripper2, 0.1, vrep.simx_opmode_oneshot)
    print(err)
    sleep(1)

    err = vrep.simxSetJointTargetPosition(clientID, first_joint, 0, vrep.simx_opmode_oneshot)
    err = vrep.simxSetJointTargetPosition(clientID, second_joint, 3, vrep.simx_opmode_oneshot)
    sleep(5)
    err = vrep.simxSetJointTargetVelocity(clientID, gripper2, -0.1, vrep.simx_opmode_oneshot)
    sleep(0.5)
    err = vrep.simxSetJointTargetVelocity(clientID, gripper2, 0, vrep.simx_opmode_oneshot)
    err = err = vrep.simxSetJointTargetPosition(clientID, second_joint, 0, vrep.simx_opmode_oneshot)
