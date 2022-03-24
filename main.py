import sim
import numpy as np
import imutils
import cv2
import matplotlib.pyplot as plt
import time
from PIL import Image
import array


def streamVisionSensor(sensor, clientID):
    if clientID != -1:
        err, vision_handle = sim.simxGetObjectHandle(clientID, sensor, sim.simx_opmode_oneshot_wait)
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, vision_handle, 0, sim.simx_opmode_streaming)

        while sim.simxGetConnectionId(clientID) != -1:
            err, resolution, image = sim.simxGetVisionSensorImage(clientID, vision_handle, 0, sim.simx_opmode_buffer)
            if err == sim.simx_return_ok:
                print("Displaying Vision Sensor")

                img = np.array(image, dtype=np.uint8)
                img.resize([resolution[0], resolution[1], 3])
                img = imutils.rotate_bound(img, 180)

                # Flip image
                img = cv2.flip(img, 1)

                # Show image and convert to RGB
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                cv2.startWindowThread()
                cv2.namedWindow("image")
                cv2.imshow('image', img_rgb)

                # Used for saving images and quitting
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
                elif key == ord('f'):
                    cv2.imwrite('positive/{}.jpg'.format(time.time()), img_rgb)
                elif key == ord('d'):
                    cv2.imwrite('negative/{}.jpg'.format(time.time()), img_rgb)

            elif err == sim.simx_return_novalue_flag:
                print("no image yet")
                pass
            else:
                print(err)

    else:
        print("Failed to connect to remote API Server")
        sim.simxFinish(clientID)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Close any running sims
    sim.simxFinish(-1)

    # Connect to Coppelia client
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    print(clientID)

    # Check connection
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")


    wheelJoints = [-1, -1, -1, -1]
    err, wheelJoints[0] = sim.simxGetObjectHandle(clientID, "rollingJoint_fl", sim.simx_opmode_blocking)
    err, wheelJoints[1] = sim.simxGetObjectHandle(clientID, "rollingJoint_rl", sim.simx_opmode_blocking)
    err, wheelJoints[2] = sim.simxGetObjectHandle(clientID, "rollingJoint_rr", sim.simx_opmode_blocking)
    err, wheelJoints[3] = sim.simxGetObjectHandle(clientID, "rollingJoint_fr", sim.simx_opmode_blocking)

    err = sim.simxSetJointTargetVelocity(clientID, wheelJoints[0], -0.5, sim.simx_opmode_oneshot)
    err = sim.simxSetJointTargetVelocity(clientID, wheelJoints[1], -0.5, sim.simx_opmode_oneshot)
    err = sim.simxSetJointTargetVelocity(clientID, wheelJoints[2], 0.5, sim.simx_opmode_oneshot)
    err = sim.simxSetJointTargetVelocity(clientID, wheelJoints[3], 0.5, sim.simx_opmode_oneshot)

    streamVisionSensor("Vision_sensor", clientID)
    print("done")
