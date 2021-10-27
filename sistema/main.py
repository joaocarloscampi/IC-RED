import sys
import rosbag
import rospy

from ZeroVelocity import ZeroVelocity
from ZeroAcceleration import ZeroAcceleration
from ExplicityComplementaryFilter import ECF
from velocity import Velocity

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import tikzplotlib

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, rotation_matrix, concatenate_matrices, quaternion_matrix

class IMU:

    def __init__(self):
        self.accel = [0, 0, 0]                                                  # Armazena os dados atuais do acelerometro
        self.gyro = [0, 0, 0]                                                   # Armazena os dados atuais do giroscópio
        self.orientation = [0, 0, 0, 0]                                         # Armazena os dados atuais de orientação em quatérnio
        self.angular_velocity_covariance = []                                   # Armazena a matriz de covariancia do giroscópio
        self.linear_acceleration_covariance = []                                # Armazena a matriz de covariancia do acelerômetro
        self.time = 0                                                           # Armazena o dado de tempo atual das medidas


class CAMERA:

    def __init__(self):
        self.orientation = [0, 0, 0, 0]                                         # Armazena os dados atuais de orientação em quatérnio
        self.time = 0                                                           # Armazena o dado de tempo atual das medidas


class VelocityMeter:
    def __init__(self):
        self.imu = IMU()                                                        # Instanciamento de um objeto IMU
        self.velocity = Velocity()                                              # Instanciamento de um objeto Velocity - Calculo de velocidade linear
        self.zeroVelocity = ZeroVelocity(self.imu)                              # Instanciamento de um objeto ZeroVelocity - Detecção de Velocidade Zero
        self.zeroAcceleration = ZeroAcceleration(self.imu, self.velocity.g)     # Instanciamento de um objeto ZeroAcceleration - Detecção de Aceleração Zero
        self.ECF = ECF()

        self.coppelia = -1                          # Variavel para trocar os eixos dos dados vindos do Coppelia: -1 troca, 1 não troca

        self.lastTime = 0                           # Variavel que salva o ultimo instante de medição
        self.vx = 0                                 # Velocidades do robo móvel
        self.vy = 0
        self.vz = 0
        self.v = 0

        self.euler = [0, 0, 0]                      # Angulos de Euler
        self.accelG = [0, 0, 0]                     # Aceleração sem a Gravidade

        self.dataZ = []                             # Dados para utilizar no plot
        self.dataGamma = []
        self.dataVelocity = []
        self.dataVx = []
        self.dataVy = []
        self.dataVz = []
        self.areStopped = []
        self.dataZeroAccel = []
        self.dataU = []
        self.dataMi = []
        self.dataMi_f = []
        self.dataAccelX = []
        self.dataAccelY = []
        self.dataAccelZ = []
        self.dataGyroX = []
        self.dataGyroY = []
        self.dataGyroZ = []
        self.time = []

    def getData(self, data):
        '''
            Função que resgata os dados publicados no Tópico selecionado
            do ROS ou do ROSBAG.
        '''
        self.imu.time = data.header.stamp.secs + data.header.stamp.nsecs*1e-9   # Tempo de captura dos dados [s]

        self.imu.orientation[0] = data.orientation.x                            # Orientação em quatérnio
        self.imu.orientation[1] = data.orientation.y
        self.imu.orientation[2] = data.orientation.z
        self.imu.orientation[3] = data.orientation.w

        self.imu.gyro[0] = data.angular_velocity.x                              # Velocidade angular [rad/s]
        self.imu.gyro[1] = data.angular_velocity.y
        self.imu.gyro[2] = data.angular_velocity.z

        self.imu.accel[0] = data.linear_acceleration.x*self.coppelia            # Aceleração linear [m/s^2]
        self.imu.accel[1] = data.linear_acceleration.y*self.coppelia
        self.imu.accel[2] = data.linear_acceleration.z*self.coppelia

        vectorCovAngular = data.angular_velocity_covariance                     # Matriz de covariancia do Giroscópio
        self.imu.angular_velocity_covariance = [ [vectorCovAngular[0], vectorCovAngular[1], vectorCovAngular[2]],
                                                 [vectorCovAngular[3], vectorCovAngular[4], vectorCovAngular[5]],
                                                 [vectorCovAngular[5], vectorCovAngular[7], vectorCovAngular[8]] ]

        vectorCovLinear = data.linear_acceleration_covariance                   # Matriz de covariancia do acelerômetro
        self.imu.linear_acceleration_covariance = [ [vectorCovLinear[0], vectorCovLinear[1], vectorCovLinear[2]],
                                                    [vectorCovLinear[3], vectorCovLinear[4], vectorCovLinear[5]],
                                                    [vectorCovLinear[5], vectorCovLinear[7], vectorCovLinear[8]] ]

        self.euler[0], self.euler[1], self.euler[2] = euler_from_quaternion(self.imu.orientation) # Angulos de Euler (convertidos do quatérnio)

    def main(self, data):
        '''
            Função main: Função que recebe os dados e chama todas as funções
            necessárias para estimar a velocidade linear
        '''
        self.getData(data)                                                      # Função que recebe os dados do ROS

        self.zeroVelocity.main()                                                # Execução do Algoritmo de Velocidade Zero
        self.zeroAcceleration.main(self.zeroVelocity.stopped, self.velocity.g)  # Execução do Algoritmo de Aceleração Zero

                                                                                # Calculo da velocidade linear
        self.velocity.calcLinearVelocity(self.imu.orientation, self.imu.accel, self.zeroVelocity.stopped, self.zeroAcceleration.notAccel, self.imu.time)
        #self.ECF.main(self.imu.orientation, self.velocity.g, self.imu.gyro)

        self.lastTime = self.imu.time                                           # Atualização do tempo de captura de dados

        self.time.append(self.imu.time)                                         # Atualiação de variaveis para realizar plots
        self.plotDataGamma("Read")
        self.plotDataAngleZ("Read")
        self.plotDataVelocity("Read")
        self.plotDataZeroAccel("Read")
        self.plotDataSensor("Read")

    def plotDataGamma(self, status, blockGraph = False):
        '''
            Função para plotar os dados do Algoritmo de Velocidade Zero
        '''

        if status == "Read":                                                    # Se o modo chamado na função for de leitura "Read"
            self.dataGamma.append(self.zeroVelocity.gamma)                      # Adiciona o valor de gamma calculado
            if self.zeroVelocity.stopped:
                self.areStopped.append(1)                                       # Se estiver parado, adicionar 1
            else:
                self.areStopped.append(0)                                       # Caso não, adicionar 0

        elif status == "Plot":                                                  # Se o modo chamado na função foi de "Plot"
            plt.rcParams.update({'font.size': 14})
            fig, axs = plt.subplots(2, sharex=True)

            axs[0].plot(velMet.time, velMet.dataGamma, label = 'Valores do gamma')
            axs[1].plot(velMet.time, velMet.areStopped, label = 'Robô Parado')  # Configurações matplotlib para plotar os dados
            axs[0].set(ylabel='T(zn)')
            axs[1].set(ylabel = 'Parado', xlabel = 'Tempo [s]')
            axs[0].grid()

            plt.show(block = blockGraph)

            # tikzplotlib.save("zeroVelocity.tex")                                # Salvando em arquivo .TeX

    def plotDataAngleZ(self, status, blockGraph = False):
        '''
            (Futuramente será trocada)
            Função para plotar os dados de angulo em Z do robô
        '''

        if status == "Read":                                                    # Se o modo chamado na função for de leitura "Read"
            self.dataZ.append(self.euler[2])                                    # Adiciona o angulo em Z

        elif status == "Plot":                                                  # Se o modo chamado na função foi de "Plot"
            plt.figure(figsize=(16,8))                                          # Configurações de plot no matplotlib
            plt.plot(velMet.time, velMet.dataZ, label = 'Valores do angulo Z')
            plt.legend()

            plt.show(block = blockGraph)

    def plotDataSensor(self, status, blockGraph = False):
        '''
            (Preferivel utilizar a função de plotRealData.py)
            Função utilizada para plot dos dados recebidos dos sensores
            acelerômetro e giroscópio
        '''

        if status == "Read":                                                    # Se o modo chamado na função for de leitura "Read"
            self.dataAccelX.append(self.imu.accel[0])                           # Adicionando os dados do acelerômetro
            self.dataAccelY.append(self.imu.accel[1])
            self.dataAccelZ.append(self.imu.accel[2])

            self.dataGyroX.append(self.imu.gyro[0])                             # Adicionando os dados do giroscópio
            self.dataGyroY.append(self.imu.gyro[1])
            self.dataGyroZ.append(self.imu.gyro[2])

        elif status == "Plot":                                                  # Se o modo chamado na função foi de "Plot"
            fig, axs = plt.subplots(2, sharex=True)                             # Configurações de plot para o matplotlib
            fig.suptitle("Dados dos sensores")
            axs[0].set_title("Acelerômetro")
            axs[1].set_title("Giroscópio")

            axs[0].plot(velMet.time, self.dataAccelX, label = 'X')
            axs[1].plot(velMet.time, self.dataGyroX, label = 'X')
            axs[1].plot(velMet.time, self.dataGyroY, label = 'Y')
            axs[1].plot(velMet.time, self.dataGyroZ, label = 'Z')

            axs[0].set(ylabel='Aceleração [m/s²]', xlim=(velMet.time[1], velMet.time[-1]))
            axs[1].set(xlabel='Tempo [s]', ylabel='Velocidade Angular [rad/s]')

            axs[1].grid()
            axs[0].grid()

            axs[0].legend()
            axs[1].legend()

            plt.show(block = blockGraph)

    def plotDataVelocity(self, status, blockGraph = False):
        '''
            Função utilizada para plotar as velocidades estimadas e esperadas
            do robô móvel utilizando o algoritmo proposto no projeto.
        '''

        if status == "Read":                                                    # Se o modo chamado na função for de leitura "Read"
            self.dataVelocity.append(self.velocity.v)                           # Adicionando os dados de velocidade
            self.dataVx.append(self.velocity.v_inercial[0])
            self.dataVy.append(self.velocity.v_inercial[1])
            self.dataVz.append(self.velocity.v_inercial[2])

        elif status == "Plot":                                                  # Se o modo chamado na função foi de "Plot"
            dataFrame = pd.read_csv('real.csv')                                 # Lendo o dataframe com os dados esperados do robô móvel
            velocityLinear = []
            for i in range(len(dataFrame["vx"])):                               # Calculo do módulo da velocidade
                v = np.sqrt(dataFrame["vx"][i] ** 2 + dataFrame["vy"][i] ** 2 + dataFrame["vz"][i] ** 2 )/100
                velocityLinear.append(v)

            plt.rcParams.update({'font.size': 14})                              # Configurações do matplotlib
            plt.figure()                                                        # Figura 1: Módulo da velocidade medida

            plt.plot(self.time, self.dataVelocity, label = 'Estimado')
            plt.plot(dataFrame['time'], velocityLinear, '-.', color = 'orange', label = "Real")

            plt.xlim(self.time[0], self.time[-1])
            plt.xlabel("Tempo [s]")
            plt.ylabel("Velocidade [m/s]")

            plt.rcParams.update({'font.size': 10})
            plt.legend()

            plt.rcParams.update({'font.size': 14})
            plt.grid()
            plt.show(block = blockGraph)

            fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)                 # Figura 2: Velocidades separadas por eixo x, y e z

            ax1.plot(self.time, self.dataVx, label = 'Estimado')
            ax2.plot(self.time, self.dataVy, label = 'Estimado')
            ax3.plot(self.time, self.dataVz, label = 'Estimado')

            ax1.plot(dataFrame['time'], dataFrame["vx"]/100 , '-.', color = 'orange', label = "Real")
            ax2.plot(dataFrame['time'], dataFrame["vy"]/100 , '-.', color = 'orange', label = "Real")
            ax3.plot(dataFrame['time'], dataFrame["vz"]/100 , '-.', color = 'orange', label = "Real")

            ax2.set(ylabel='Velocidade [m/s]')
            ax3.set(xlabel='Tempo [s]')
            ax3.set(xlim=(self.time[0], self.time[-1]))

            ax1.grid()
            ax2.grid()
            ax3.grid()

            plt.rcParams.update({'font.size': 10})
            ax2.legend()
            plt.rcParams.update({'font.size': 14})
            plt.show(block = blockGraph)

            # tikzplotlib.save("velocity_zeroAccel.tex")                          # Salvando em arquivo .TeX

    def plotDataZeroAccel(self, status, blockGraph = False):
        if status == "Read":
            self.dataMi.append(self.zeroAcceleration.mi)
            self.dataU.append(self.zeroAcceleration.ya)
            self.dataMi_f.append(self.zeroAcceleration.mi_f)
            if self.zeroAcceleration.notAccel:
                self.dataZeroAccel.append(1)
            else:
                self.dataZeroAccel.append(0)
        elif status == "Plot":
            fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, sharex=True)
            fig.suptitle('Aceleração Zero')

            ax1.set(ylabel='mi')
            ax2.set(ylabel='u')
            ax3.set(ylabel='mi_f')
            ax4.set(ylabel='notAccel')
            ax5.set(xlabel='Tempo [s]', ylabel='Vel0')

            ax1.set(xlim = (velMet.time[1], velMet.time[-1]), ylim = (0, 2))

            ax1.plot(velMet.time, self.dataMi)
            #ax2.plot(velMet.time, self.dataU)
            ax2.plot(velMet.time, self.dataU)
            ax3.plot(velMet.time, self.dataMi_f)
            ax4.plot(velMet.time, self.dataZeroAccel)
            ax5.plot(velMet.time, self.areStopped)
            plt.show(block = blockGraph)

    def plotAllData(self):
        '''
            Função bagunçada...
        '''
        dataFrame = pd.read_csv('real.csv')

        #fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, sharex=True)
        #fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
        plt.rcParams.update({'font.size': 14})
        fig, (ax1, ax2) = plt.subplots(2, sharex=True)

        #ax1.set_title('Angulo Z')
        #ax2.set_title('Módulo da velocidade')
        #ax3.set_title('Velocidade Zero')

        #ax1.set(ylabel='Angulo [rad]')
        #ax2.set(ylabel='Velocidade [m/s]')
        #ax3.set(ylabel='Parado')

        ax1.set(ylabel='Velocidade [m/s]')
        ax2.set(ylabel = 'Parado', xlabel = 'Tempo [s]')

        #ax4.set(ylabel='Vx [m/s]')
        #ax5.set(ylabel='Vy [m/s]')
        #ax6.set(xlabel='Tempo [s]', ylabel='Vz [m/s]')
        #ax3.set(xlabel='Tempo [s]', ylabel='Parado')
        #ax1.plot(velMet.time, velMet.dataZ, '-.', color = 'orange', label = "Real")
        #ax2.plot(self.time, self.dataVelocity, label = "Estimado")
        #ax3.plot(velMet.time, velMet.areStopped, label = "Estimado")

        ax1.plot(self.time, self.dataVelocity, label = "Estimado")
        ax2.plot(velMet.time, velMet.areStopped, label = "Estimado")

        velocityLinear = []
        for i in range(len(dataFrame["vx"])):
            v = np.sqrt(dataFrame["vx"][i] ** 2 + dataFrame["vy"][i] ** 2 + dataFrame["vz"][i] ** 2 )/100
            velocityLinear.append(v)
        ax1.plot(dataFrame['time'], velocityLinear, '-.', label =  'Real')
        ax2.plot(dataFrame['time'], dataFrame["zeroVelocity"],  '-.', label = 'Real')
        ax1.grid()
        ax2.grid()
        plt.rcParams.update({'font.size': 10})
        ax1.legend()
        #fig.suptitle("Velocidade com Algoritmo Velocidade-Zero")

        #ax4.plot(velMet.time, self.dataVx, color = 'blue', label = "Vx")
        #ax5.plot(velMet.time, self.dataVy, color = 'blue', label = "Vy")
        #ax6.plot(velMet.time, self.dataVz, color = 'blue', label = "Vz")
        #ax4.plot(dataFrame['time'], dataFrame["vx"]/100 , '-.', color = 'orange', label = "Vx")
        #ax5.plot(dataFrame['time'], dataFrame["vy"]/100 , '-.', color = 'orange', label = "Vy")
        #ax6.plot(dataFrame['time'], dataFrame["vz"]/100 , '-.', color = 'orange', label = "Vz")
        plt.show(block = True)


def listener(velMet, topic):
    '''
        Função listener: Função utilizada para conexão no ROS com os tópicos
        passados como parametro na execução do código
    '''

    # Dois tópicos usados até o momento:
    #   /phone1/android/imu
    #   /sensor

    rospy.init_node('reciver', anonymous=True)

    rospy.Subscriber(topic, Imu, velMet.main)

    rospy.spin() # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':

    velMet = VelocityMeter()                                                    # Instancia o objeto do medidor de velocidade

    topic = sys.argv[1]                                                         # Obtem o nome do topico passado por parametro
    listener(velMet, topic)                                                     # Inicia a comunicação ROS

    # Após o código ser executado, as funções de plot são chamadas
    
    #velMet.plotDataGamma("Plot", blockGraph = False)
    #velMet.plotDataAngleZ("Plot", blockGraph = False)
    velMet.plotDataVelocity("Plot", blockGraph = True)
    #velMet.plotDataZeroAccel("Plot", blockGraph = True)
    #velMet.plotDataSensor("Plot", blockGraph=True)
    #velMet.plotAllData()
