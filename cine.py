import numpy as np
import time
import math


def rpy(x, y, z, w):
    """ Calculo de Roll, Pitch e Yaw de um Quatérnio. """

    roll = math.atan2(2 * ((w * x) + (y * z)), 1 - 2 * ((x ** 2) + (y ** 2)))

    if abs((2 * ((w * y) - (z * x)))) >= 1:
        pitch = np.pi / 2
    else:
        pitch = math.asin(2 * ((w * y) - (z * x)))

    yaw = math.atan2(2 * ((w * z) + (x * y)), 1 - 2 * ((y ** 2) + (z ** 2)))

    return roll, pitch, yaw


def get_joints(client, topic):
    returnCode, handle = client.simxGetObjectHandle('Dummy', topic)
    dummy = handle
    returnCode, dummypos = client.simxGetObjectPosition(dummy, -1, topic)

    returnCode, handle = client.simxGetObjectHandle('joint1', topic)
    j1 = handle
    returnCode, j1pos = client.simxGetJointPosition(j1, topic)

    returnCode, handle = client.simxGetObjectHandle('joint2', topic)
    j2 = handle
    returnCode, j2pos = client.simxGetJointPosition(j2, topic)

    returnCode, handle = client.simxGetObjectHandle('joint3', topic)
    j3 = handle
    returnCode, j3pos = client.simxGetJointPosition(j3, topic)

    returnCode, handle = client.simxGetObjectHandle('joint4', topic)
    j4 = handle
    returnCode, j4pos = client.simxGetJointPosition(j4, topic)

    returnCode, handle = client.simxGetObjectHandle('joint5', topic)
    j5 = handle
    returnCode, j5pos = client.simxGetJointPosition(j5, topic)

    returnCode, handle = client.simxGetObjectHandle('joint6', topic)
    j6 = handle
    returnCode, j6pos = client.simxGetJointPosition(j6, topic)

    return j1, j2, j3, j4, j5, j6, dummy


def get_pose(client, topic):
    returnCode, handle = client.simxGetObjectHandle('Dummy', topic)
    dummy = handle
    returnCode, d_pose = client.simxGetObjectPose(dummy, -1, topic)

    returnCode, handle = client.simxGetObjectHandle('joint1', topic)
    j1 = handle
    returnCode, j1_pose = client.simxGetObjectPose(j1, -1, topic)

    returnCode, handle = client.simxGetObjectHandle('joint2', topic)
    j2 = handle
    returnCode, j2_pose = client.simxGetObjectPose(j2, -1, topic)
    return j1_pose, j2_pose, d_pose


def set_quaternion(x, y, z, w, handle, client, topic):
    returnCode, handle_aux = client.simxGetObjectParent(handle, topic)
    client.simxSetObjectQuaternion(handle, handle_aux, [x, y, z, w], topic)


def DK_planar(theta1, theta2, j1, j2, dummy, client, topic):
    # Cálculo da posição do End Effector através da cinemática direta
    q1 = theta1  # # Ângulo da junta 1
    q2 = theta2  # * np.pi/180  # Ângulo da junta 2
    x = 0.2 * np.cos(q1 + q2) + 0.3 * np.cos(q1)  # Solução da cinemática direta para o valor de x do End Effector
    y = 0.2 * np.sin(q1 + q2) + 0.3 * np.sin(q1)  # Solução da cinemática direta para o valor de y do End Effector]
    z = 0.0000  # Robo planar
    poscine = [round(x, 4), round(y, 4), round(z, 4)]
    print('Posição Cinemática Direta:' + str(poscine))
    returnCode = client.simxSetJointTargetPosition(j1, q1, topic)
    returnCode = client.simxSetJointTargetPosition(j2, q2, topic)

    # time.sleep(1/1000)

    [returnCode, pos] = client.simxGetObjectPosition(dummy, -1, topic)
    posround = [round(num, 4) for num in pos]
    returnCode = client.simxSetJointTargetPosition(j1, q1, topic)
    returnCode = client.simxSetJointTargetPosition(j2, q2, topic)

    # time.sleep(1/1000)

    [returnCode, pos] = client.simxGetObjectPosition(dummy, -1, topic)
    posround = [round(num, 4) for num in pos]
    print('Posição Real no Simulator:' + str(posround))
    return posround


def IK_planar(j1, j2, client, topic):
    returnCode, handle = client.simxGetObjectHandle('Target', topic)
    target = handle
    while True:
        returnCode, targetpos = client.simxGetObjectPosition(target, -1, topic)
        x_t, y_t = targetpos[0], targetpos[1]
        q2 = np.arccos((x_t ** 2 + y_t ** 2 - 0.13) / 0.12)
        for q1 in np.arange(-np.pi, np.pi, 0.0001):
            if abs(0.2 * np.cos(q1 + q2) + 0.3 * np.cos(q1) - x_t) < 0.001 and abs(
                    0.2 * np.sin(q1 + q2) + 0.3 * np.sin(q1) - y_t) < 0.001:
                break
        if -np.pi <= q2 <= np.pi:
            returnCode = client.simxSetJointTargetPosition(j1, q1+(np.pi/2), topic)
            returnCode = client.simxSetJointTargetPosition(j2, q2, topic)
            q1 = q1 * (180 / np.pi)
            q2 = q2 * (180 / np.pi)
            print("Ângulo da Junta 1: " + str(round(q1, 2)) + "°" + "\n"
                                                                    "Ângulo da Junta 2: " + str(round(q2, 2)) + "°")
        else:
            print("Target está fora do envelope de trabalho!" + '\n' + "O robô permanecerá na última posição válida.")
