from __future__ import print_function
import b0RemoteApi
from myo.utils import TimeInterval
from collections import deque
import myo
import cine
import numpy as np
import math
import time

global number_of_samples
global data_array
global data_array_ori
global data_array_pose
global giro
global erro

erro = False
number_of_samples = 5000
calibra = [0, 0, 0, 0]
ultima = [0]
data_array = []
data_array_ori = np.zeros(number_of_samples)
data_array_pose = []
giro = float(0)

class Listener(myo.DeviceListener):

    def __init__(self, n, erro):
        self.interval = TimeInterval(None, 0.05)
        self.orientation = None
        self.acceleration = None
        self.gyroscope = None
        self.emg = None
        self.pose = myo.Pose.rest
        self.emg_enabled = True
        self.locked = False
        self.rssi = None
        self.n = n
        self.t = deque(maxlen=n)
        self.emg_data_queue = deque(maxlen=n)
        self.pose_data_queue = deque(maxlen=n)
        self.orientation_data_queue = deque(maxlen=n)
        self.orientation_w_queue = deque(maxlen=n)
        self.orientation_x_queue = deque(maxlen=n)
        self.orientation_y_queue = deque(maxlen=n)
        self.orientation_z_queue = deque(maxlen=n)
        self.acceleration_data_queue = deque(maxlen=n)
        self.acceleration_x_queue = deque(maxlen=n)
        self.acceleration_y_queue = deque(maxlen=n)
        self.acceleration_z_queue = deque(maxlen=n)
        self.gyroscope_data_queue = deque(maxlen=n)
        self.gyroscope_x_queue = deque(maxlen=n)
        self.gyroscope_y_queue = deque(maxlen=n)
        self.gyroscope_z_queue = deque(maxlen=n)
        self.erro = erro
        self.roll = None
        self.pitch = None
        self.yaw = None

    def on_paired(self, event):
        print("Hello, {}!".format(event.device_name))
        event.device.vibrate(myo.VibrationType.short)
        event.device.stream_emg(True)
        return

    def on_unpaired(self, event):
        return False  # Stop the hub

    def on_orientation(self, event):
        #Retorna a orientação atual do sistema
        self.orientation = event.orientation
        if self.erro == False:
            self.roll,self.pitch,self.yaw = cine.rpy(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)
            self.erro = True
        else:
           pass
        rpy = cine.rpy(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)
        client.simxSetJointTargetPosition(j1, rpy[2]-self.yaw, topic)
        client.simxSetJointTargetPosition(j2, rpy[0]-self.roll, topic)
        client.simxSetJointTargetPosition(j3, rpy[1]-self.pitch, topic)

    def on_pose(self, event):
        self.pose = event.pose
        time.sleep(1 / 1000)
        
        if self.pose == myo.Pose.fingers_spread:
            #abre garra
            client.simxSetJointTargetPosition(j5, np.pi/15, topic)
            client.simxSetJointTargetPosition(j6, -np.pi/15, topic)
            return print("Mão aberta detectada - Garra desativada!")

        if self.pose == myo.Pose.fist:
            #fecha garra
            client.simxSetJointTargetPosition(j5, -np.pi/3.2, topic)
            client.simxSetJointTargetPosition(j6, np.pi/3.2, topic)
            return print("Punho fechado detectado - Garra ativada!")

    def on_emg(self, event):
        self.emg = event.emg
        self.emg_data_queue.append(event.emg)
        # print(self.emg)
        if len(list(self.emg_data_queue)) >= number_of_samples:
            data_array.append(list(self.emg_data_queue))
            self.emg_data_queue.clear()
            return False


# Cálculo da posição do End Effector através da cinemática direta
#poscine = cine.DK_planar(1.50,0, j1, j2, dummy, client, topic)
# Cálculo da cinemática inversa para o robô planar
#cine.IK_planar(j1, j2, client, topic)

if __name__ == '__main__':
    myo.init(sdk_path='C:/Users/rcass/Downloads/myo-sdk-win-0.9.0/myo-sdk-win-0.9.0')
    hub = myo.Hub()
    listener = Listener(number_of_samples,erro)
    client = b0RemoteApi.RemoteApiClient('client', 'b0RemoteApi')
    simu = client.simxDefaultPublisher()
    topic = client.simxServiceCall()
    client.simxStartSimulation(simu)
    # Obtenção dos handles/manejadores associados aos elementos do robô
    j1, j2, j3, j4, j5, j6, dummy = cine.get_joints(client, topic)
    returnCode = client.simxSetJointTargetPosition(j1, 0, topic)
    returnCode = client.simxSetJointTargetPosition(j2, 0, topic)
    returnCode = client.simxSetJointTargetPosition(j3, 0, topic)
    returnCode = client.simxSetJointTargetPosition(j4, 0, topic)
    returnCode = client.simxSetJointTargetPosition(j5, 0, topic)
    returnCode = client.simxSetJointTargetPosition(j6, 0, topic)
    #j1_pose, j2_pose, d_pose = cine.get_pose(client, topic)
    while hub.run(listener.on_event, 500):
        pass
    print('Bye, bye!')
    client.simxStopSimulation(simu)