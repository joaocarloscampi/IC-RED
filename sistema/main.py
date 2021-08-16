import sys
import rosbag
import ZeroVelocity
import numpy as np
import matplotlib.pyplot as plt

class IMU:

    def __init__(self):

        self.accel = [0, 0, 0]                                                  # Armazena os dados atuais do acelerometro
        self.gyro = [0, 0, 0]                                                   # Armazena os dados atuais do giroscópio
        self.orientation = [0, 0, 0, 0]                                         # Armazena os dados atuais de orientação em quatérnio
        self.time = 0                                                           # Armazena o dado de tempo atual das medidas


class CAMERA:

    def __init__(self):

        self.orientation = [0, 0, 0, 0]                                         # Armazena os dados atuais de orientação em quatérnio
        self.time = 0                                                           # Armazena o dado de tempo atual das medidas


if __name__ == '__main__':

    if(len(sys.argv) > 1):
        bag = sys.argv[1]
    else:
        print("Insira o nome do arquivo .bag como parâmetro")
        sys.exit()

    bag_in = rosbag.Bag(bag)
    imu = IMU()
    zeroVelocity = ZeroVelocity.ZeroVelocity(imu)
    dataZ = []
    time = []

    for topic, msg, t in bag_in.read_messages(["/sensor"]):

        imu.time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
        imu.orientation[0] = msg.orientation.w
        imu.orientation[1] = msg.orientation.x
        imu.orientation[2] = msg.orientation.y
        imu.orientation[3] = msg.orientation.z
        imu.gyro[0] = msg.angular_velocity.x
        imu.gyro[1] = msg.angular_velocity.y
        imu.gyro[2] = msg.angular_velocity.z
        imu.accel[0] = msg.linear_acceleration.x
        imu.accel[1] = msg.linear_acceleration.y
        imu.accel[2] = msg.linear_acceleration.z

        #print("imu orientation: " + str(imu.orientation))
        zeroVelocity.ZeroVelocity()
        #x, y, z = quaternion_to_euler_angle_vectorized1(imu.orientation[0], imu.orientation[1], imu.orientation[2], imu.orientation[3])
        z = np.arctan2( 2*(imu.orientation[0]*imu.orientation[3] + imu.orientation[1]*imu.orientation[2]), 1-2*(imu.orientation[2]**2 + imu.orientation[3]**2) )
        dataZ.append(z)
        time.append(imu.time)

    plt.figure(figsize=(16,8))
    plt.plot(time, dataZ, label = 'Valores do angulo Z')
    plt.legend()
    plt.show(block = False)

    zeroVelocity.plotData()
