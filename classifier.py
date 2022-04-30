import sim
import numpy as np
import imutils
import cv2
import time
import os


def verify(clientID, sensor, wheel_handles, detected_obj):
    err, vision_handle = sim.simxGetObjectHandle(clientID, sensor, sim.simx_opmode_oneshot_wait)
    time.sleep(1)

    # Rotate left and check again
    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[0], 0.2, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[3], -0.2, sim.simx_opmode_streaming)
    time.sleep(0.5)

    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[0], 0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[3], 0, sim.simx_opmode_streaming)
    time.sleep(0.5)

    if stream_vision_sensor(sensor, clientID)[1] != detected_obj[1]:
        return False

    # Rotate right and check again
    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[0], -0.2, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[3], 0.2, sim.simx_opmode_streaming)
    time.sleep(1)

    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[0], 0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, wheel_handles[3], 0, sim.simx_opmode_streaming)
    time.sleep(0.5)

    if stream_vision_sensor(sensor, clientID)[1] != detected_obj[1]:
        return False

    return True


def stream_vision_sensor(sensor, clientID):
    # Load classifiers
    blue_classifier = cv2.CascadeClassifier('Blue_Classifier/classify/cascade.xml')
    red_classifier = cv2.CascadeClassifier('Red_Classifier/classify/cascade.xml')

    if clientID != -1:
        err, vision_handle = sim.simxGetObjectHandle(clientID, sensor, sim.simx_opmode_oneshot_wait)
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, vision_handle, 0, sim.simx_opmode_streaming)

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

            annotate_image = img_rgb.copy()
            red_image = img_rgb.copy()

            # Load image into classifiers
            blue_output = blue_classifier.detectMultiScale(annotate_image)
            red_output = red_classifier.detectMultiScale(red_image)

            # Draw rectangles on image
            for (x, y, w, h) in blue_output:
                cv2.rectangle(annotate_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(annotate_image, 'blue cube', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

            for (x, y, w, h) in red_output:
                cv2.rectangle(annotate_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(annotate_image, 'red cube', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

            cv2.imshow('Detection Output', annotate_image)
            # cv2.imshow('Red output', red_image)

            # Used for saving images and quitting
            key = cv2.waitKey(1)
            if key == ord('q'):
                quit()
            elif key == ord('f'):
                cv2.imwrite('Blue_Classifier/positive_new/{}.jpg'.format(time.time()), img_rgb)
            elif key == ord('d'):
                cv2.imwrite('Blue_Classifier/negative/{}.jpg'.format(time.time()), img_rgb)

            # If any objects are detected return True
            if len(blue_output) != 0 or len(red_output) != 0:
                if len(blue_output) != 0:
                    return [True, "blue"]
                else:
                    return [True, "red"]
            return [False, ""]

        elif err == sim.simx_return_novalue_flag:
            print("no image yet")
        else:
            print(err)

    else:
        print("Failed to connect to remote API Server")
        sim.simxFinish(clientID)
