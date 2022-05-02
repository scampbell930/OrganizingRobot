import sim as vrep # access all the VREP elements
import numpy as np
import time
import math
#Reduce false negatives for checking if there is an object at a point
def check_for_object() -> bool:
    err_code, collision1 = vrep.simxCheckCollision(clientID,dummy_handle,vrep.sim_handle_all,vrep.simx_opmode_blocking )
    err_code, collision2 = vrep.simxCheckCollision(clientID,dummy_handle,vrep.sim_handle_all,vrep.simx_opmode_blocking )
    if( collision1 == collision2):
        return collision1
    return check_for_object()


def free_line(pos1, pos2):
    vector = [pos2[0] - pos1[0], pos2[1] - pos1[1]]
    i = 0

    while(i <= 1):
        position = [i*vector[0] + pos1[0], i*vector[1] + pos1[1]]
        err_code = vrep.simxSetObjectPosition(clientID, dummy_handle,-1,[position[0],position[1],0.2], vrep.simx_opmode_blocking )
        collision = check_for_object()
        if(collision):
            return False
        i += 0.25
    return True       

        

if __name__ == "__main__":
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    vertexes = []
    if clientID != -1:
        err_code,dummy_handle = vrep.simxGetObjectHandle(clientID, "/test_cube", vrep.simx_opmode_blocking)
        x_pos = -5

        while(x_pos < 5):
            y_pos = -5
            while(y_pos < 5):
                err_code = vrep.simxSetObjectPosition(clientID, dummy_handle,-1,[x_pos,y_pos,0.2], vrep.simx_opmode_blocking )
                collision = check_for_object()
                if(not collision):
                    f = open("nodes.txt", "a")
                    f.write(str(round(x_pos,2)) + " " + str(round(y_pos,2)) + "\n")
                    vertexes.append([x_pos,y_pos])
                y_pos += 0.8
            x_pos += 0.8        

    for i in range(len(vertexes)):
        for j in range(i+1,len(vertexes)): 
            if(math.dist([vertexes[i][0], vertexes[i][1]],[vertexes[j][0],vertexes[j][1]]) <= 2):
                if(free_line(vertexes[i],vertexes[j])):
                    f = open("edges.txt", "a")
                    f.write(str(i) + ' ' + str(j) + ' ' + str(round(math.dist([vertexes[i][0], vertexes[i][1]],[vertexes[j][0],vertexes[j][1]]) , 2)) + '\n')
