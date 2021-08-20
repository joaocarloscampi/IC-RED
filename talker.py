#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped


class Publisher():


    def __init__(self):
        self.timeIntegerSendCamera = 0                                          # Variavel de controle para envio de informações da camera
        self.rateCamera = 30                                                    # Taxa de atualização da camera


    def startPublisher(self):
        '''
            Função que inicia o nó utilizado pelo sistema e configura as mensagens
            publicadas por ela, com seus tópicos
        '''


        rospy.loginfo("Initializing imu publisher")
        self.imu_pub = rospy.Publisher('sensor', Imu, queue_size=5)             # Inicia o objeto que publicará mensagens Imu
        rospy.loginfo("Publishing Imu at: " + self.imu_pub.resolved_name)


        rospy.loginfo("Initializing vision publisher")
        self.camera_pub = rospy.Publisher('camera', PoseStamped, queue_size=5)  # Inicia o objeto que publicará mensagens PoseStamped
        rospy.loginfo("Publishing vision at: " + self.camera_pub.resolved_name)


        rospy.init_node('Simulator', anonymous=True)                            # Inicia o nó do sistema


    def talkerSimulator(self, time, quaternion, angular_velocity, covarianceGyro, linear_acceleration, covarianceAccel, position):
        '''
            Função que publica os dados recebidos como parametros nas mensagens
            IMU e PoseStamped no ROS
        '''

        if not rospy.is_shutdown():
            self.i = Imu()                                                      # Instancia da mensagem Imu
            self.ps = PoseStamped()                                             # Instancia da mensagem PoseStamped

            #   Calculo:                        tempo
            #             Se parte inteira de  -------  é diferente da ultima parte inteira, então envia os dados
            #                                 frequencia
            if ( int( time / (1/self.rateCamera) ) !=  self.timeIntegerSendCamera): # Verifica se ja é necessário mandar outro frame da camera
                self.ps.header.stamp.secs = int(time)                           # Preenchendo os atributos de PoseStamped
                self.ps.header.stamp.nsecs = int((time - int(time))*10**9)
                self.ps.header.frame_id = 'camera'
                self.q = Quaternion()
                self.q.x = quaternion[0]
                self.q.y = quaternion[1]
                self.q.z = quaternion[2]
                self.q.w = quaternion[3]
                self.ps.pose.orientation = self.q
                self.ps.pose.position.x = position[0]
                self.ps.pose.position.y = position[1]
                self.ps.pose.position.z = position[2]

                rospy.loginfo(self.ps)
                self.camera_pub.publish(self.ps)                                # Publicando a mensagem PoseStamped

                self.timeIntegerSendCamera = int( time / (1/self.rateCamera) )  # Atualização da variavel de controle da camera

            self.i.header.stamp.secs = int(time)                                # Preenchendo os atributos da Imu
            self.i.header.stamp.nsecs = int((time - int(time))*10**9)
            self.i.header.frame_id = 'imurobot'
            self.q = Quaternion()
            self.q.x = quaternion[0]
            self.q.y = quaternion[1]
            self.q.z = quaternion[2]
            self.q.w = quaternion[3]
            self.v = Vector3()
            self.v.x = angular_velocity[0]
            self.v.y = angular_velocity[1]
            self.v.z = angular_velocity[2]
            self.a = Vector3()
            self.a.x = linear_acceleration[0]
            self.a.y = linear_acceleration[1]
            self.a.z = linear_acceleration[2]
            self.i.orientation = self.q
            self.i.angular_velocity = self.v
            self.i.linear_acceleration = self.a
            self.i.angular_velocity_covariance = covarianceGyro
            self.i.linear_acceleration_covariance = covarianceAccel

            rospy.loginfo(self.i)
            self.imu_pub.publish(self.i)                                        # Publicando a mensagem Imu
