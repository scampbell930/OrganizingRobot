import sim as vrep


def pickup_object(clientID):
    err_code, first_joint = vrep.simxGetObjectHandle(clientID, "/youBot/youBotArmJoint0/youBotArmJoint1",
                                                     vrep.simx_opmode_blocking)
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
