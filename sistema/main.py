import sys
import rosbag
import rospy
from ZeroVelocity import ZeroVelocity
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class IMU:

    def __init__(self):

        self.accel = [0, 0, 0]                                                  # Armazena os dados atuais do acelerometro
        self.gyro = [0, 0, 0]                                                   # Armazena os dados atuais do giroscópio
        self.orientation = [0, 0, 0, 0]                                         # Armazena os dados atuais de orientação em quatérnio
        self.angular_velocity_covariance = []
        self.linear_acceleration_covariance = []
        self.time = 0                                                           # Armazena o dado de tempo atual das medidas


class CAMERA:

    def __init__(self):

        self.orientation = [0, 0, 0, 0]                                         # Armazena os dados atuais de orientação em quatérnio
        self.time = 0                                                           # Armazena o dado de tempo atual das medidas


class VelocityMeter:
    def __init__(self):
        self.imu = IMU()
        self.zeroVelocity = ZeroVelocity(self.imu)

        self.dataZ = []
        self.dataGamma = []
        self.areStopped = []
        self.time = []

    def main(self, data):
        self.imu.time = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
        self.imu.orientation[0] = data.orientation.w
        self.imu.orientation[1] = data.orientation.x
        self.imu.orientation[2] = data.orientation.y
        self.imu.orientation[3] = data.orientation.z
        self.imu.gyro[0] = data.angular_velocity.x
        self.imu.gyro[1] = data.angular_velocity.y
        self.imu.gyro[2] = data.angular_velocity.z
        self.imu.accel[0] = data.linear_acceleration.x
        self.imu.accel[1] = data.linear_acceleration.y
        self.imu.accel[2] = data.linear_acceleration.z
        vectorCovAngular = data.angular_velocity_covariance
        self.imu.angular_velocity_covariance = [ [vectorCovAngular[0], vectorCovAngular[1], vectorCovAngular[2]],
                                                 [vectorCovAngular[3], vectorCovAngular[4], vectorCovAngular[5]],
                                                 [vectorCovAngular[5], vectorCovAngular[7], vectorCovAngular[8]] ]
        vectorCovLinear = data.linear_acceleration_covariance
        self.imu.linear_acceleration_covariance = [ [vectorCovLinear[0], vectorCovLinear[1], vectorCovLinear[2]],
                                                    [vectorCovLinear[3], vectorCovLinear[4], vectorCovLinear[5]],
                                                    [vectorCovLinear[5], vectorCovLinear[7], vectorCovLinear[8]] ]

        self.zeroVelocity.main()

        self.time.append(self.imu.time)

        self.plotDataGamma("Read")
        self.plotDataAngleZ("Read")


    def plotDataGamma(self, status, blockGraph = False):
        if status == "Read":
            self.dataGamma.append(self.zeroVelocity.gamma)
            if self.zeroVelocity.gamma < 1e-4:
                self.areStopped.append(1)
            else:
                self.areStopped.append(0)

        elif status == "Plot":
            fig, axs = plt.subplots(2)
            fig.suptitle('Dados Gamma')
            axs[0].plot(velMet.time, velMet.dataGamma, label = 'Valores do gamma')
            axs[1].plot(velMet.time, velMet.areStopped, label = 'Robô Parado')

    def plotDataAngleZ(self, status, blockGraph = False):
        if status == "Read":
            z = np.arctan2( 2*(self.imu.orientation[0]*self.imu.orientation[3] + self.imu.orientation[1]*self.imu.orientation[2]), 1-2*(self.imu.orientation[2]**2 + self.imu.orientation[3]**2) )
            self.dataZ.append(z)
        elif status == "Plot":
            plt.figure(figsize=(16,8))
            plt.plot(velMet.time, velMet.dataZ, label = 'Valores do angulo Z')
            plt.legend()
            plt.show(block = blockGraph)


def listener(velMet):

    rospy.init_node('reciver', anonymous=True)

    rospy.Subscriber('sensor', Imu, velMet.main)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    velMet = VelocityMeter()
    listener(velMet)

    # Temporário - Após o código executar ele irá plotar o gráfico
    velMet.plotDataGamma("Plot")
    velMet.plotDataAngleZ("Plot", blockGraph = True)
