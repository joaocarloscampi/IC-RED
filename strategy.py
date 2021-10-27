from numpy import pi,array_equal,array,tan
from simClasses import Ball,Robot,Target,Obstacle
import action
from time import sleep, time
import imu
from talker import Publisher

from path import Path
import pandas as pd

class StrategyTesting:
    def __init__(self):
        self.ball=Ball()
        self.redRob=Robot()
        self.greenRob=Robot()
        self.pinkRob=Robot()
        self.IMUgreenRob=imu.IMU(self.greenRob, '#0') # Adicionado para a IC
        self.arrayFunctions = [self.pinkRob, self.redRob, self.greenRob] # [Goalkeeper, Defender, Attacker]

        self.xPos = []
        self.yPos = []
        self.zPos = []
        self.vx = []
        self.vy = []
        self.vz = []
        self.wx = []
        self.wy = []
        self.wz = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.zeroVelocity = []
        self.time = []
        self.dataSave = False

        self.sensorAccelX = []
        self.sensorAccelY = []
        self.sensorAccelZ = []
        self.sensorGyroX = []
        self.sensorGyroY = []
        self.sensorGyroZ = []
        self.sensorAccelNoiseX = []
        self.sensorAccelNoiseY = []
        self.sensorAccelNoiseZ = []
        self.sensorGyroNoiseX = []
        self.sensorGyroNoiseY = []
        self.sensorGyroNoiseZ = []


    def simConnect(self,clientID):
        self.clientID=clientID

        self.IMUgreenRob.getClientID(clientID) # Adicionado para a IC
        self.ball.simConnect(self.clientID,'ball')
        self.redRob.simConnect(self.clientID,'soccerRob_pos','soccerRob_teamMarker','soccerRob_IDMarker','leftMotor','rightMotor', 'soccerRob_pos')
        self.greenRob.simConnect(self.clientID,'soccerRob_pos#0','soccerRob_teamMarker#0','soccerRob_IDMarker#0','leftMotor#0','rightMotor#0', 'soccerRob_dyn#0')
        self.pinkRob.simConnect(self.clientID,'soccerRob_pos#1','soccerRob_teamMarker#1','soccerRob_IDMarker#1','leftMotor#1','rightMotor#1', 'soccerRob_dyn#1')

        if self.redRob.simCheckConnection():
            print('RedRob ready to play!')
        else:
            print('RedRob not found...')
        if self.greenRob.simCheckConnection():
            print('GreenRob ready to play!')
        else:
            print('GreenRob not found...')
        if self.pinkRob.simCheckConnection():
            print('pinkRob ready to play!')
        else:
            print('pinkRob not found...')
        if self.ball.simCheckConnection():
            print('Ball ready to play!\n')
        else:
            print('Ball not found...\n')
        self.redRob.simGetPose('infLeft_cornor')
        self.greenRob.simGetPose('infLeft_cornor')
        self.pinkRob.simGetPose('infLeft_cornor')
        self.ball.simGetPose('infLeft_cornor')

        i=0
        while self.redRob.xPos == 0 or self.greenRob.xPos == 0 or self.pinkRob.xPos == 0 or self.ball.xPos == 0 :
            self.redRob.simGetPose('infLeft_cornor')
            self.greenRob.simGetPose('infLeft_cornor')
            self.pinkRob.simGetPose('infLeft_cornor')
            self.ball.simGetPose('infLeft_cornor')
            i+=1
        print(i,'tentativas até pegar as posições\n')

        self.publisher = Publisher()                                            # Instancia do objeto Publisher ROS
        self.publisher.startPublisher()                                         # Inicio do Publisher
        self.path = Path(self.IMUgreenRob)

    def play(self):
        self.pinkRob.simGetPose('infLeft_cornor')
        self.redRob.simGetPose('infLeft_cornor')
        self.greenRob.simGetPose('infLeft_cornor')
        self.ball.simGetPose('infLeft_cornor')

        status = self.path.path1()


        self.IMUgreenRob.simulateIMU()

        self.xPos.append(self.greenRob.xPos)
        self.yPos.append(self.greenRob.yPos)
        self.zPos.append(self.greenRob.zPos)
        self.vx.append(self.IMUgreenRob.linearVelocity[0])
        self.vy.append(self.IMUgreenRob.linearVelocity[1])
        self.vz.append(self.IMUgreenRob.linearVelocity[2])
        self.wx.append(self.IMUgreenRob.angularVelocity[0])
        self.wy.append(self.IMUgreenRob.angularVelocity[1])
        self.wz.append(self.IMUgreenRob.angularVelocity[2])
        self.qw.append(self.greenRob.quaternion[3])
        self.qx.append(self.greenRob.quaternion[0])
        self.qy.append(self.greenRob.quaternion[1])
        self.qz.append(self.greenRob.quaternion[2])
        self.time.append(self.IMUgreenRob.nowTime)

        self.sensorGyroX.append(self.IMUgreenRob.GyroSensor[0])
        self.sensorGyroY.append(self.IMUgreenRob.GyroSensor[1])
        self.sensorGyroZ.append(self.IMUgreenRob.GyroSensor[2])
        self.sensorAccelX.append(self.IMUgreenRob.accelSensor[0])
        self.sensorAccelY.append(self.IMUgreenRob.accelSensor[1])
        self.sensorAccelZ.append(self.IMUgreenRob.accelSensor[2])
        self.sensorAccelNoiseX.append(self.IMUgreenRob.accelSensorSimulate[0])
        self.sensorAccelNoiseY.append(self.IMUgreenRob.accelSensorSimulate[1])
        self.sensorAccelNoiseZ.append(self.IMUgreenRob.accelSensorSimulate[2])
        self.sensorGyroNoiseX.append(self.IMUgreenRob.GyroSensorSimulate[0])
        self.sensorGyroNoiseY.append(self.IMUgreenRob.GyroSensorSimulate[1])
        self.sensorGyroNoiseZ.append(self.IMUgreenRob.GyroSensorSimulate[2])


        position = [self.greenRob.xPos, self.greenRob.yPos, self.greenRob.zPos]
        covarianceAccel = [self.IMUgreenRob.VAR_a[0], 0, 0, 0, self.IMUgreenRob.VAR_a[1], 0, 0, 0, self.IMUgreenRob.VAR_a[2]]
        covarianceGyro = [self.IMUgreenRob.VAR_g[0], 0, 0, 0, self.IMUgreenRob.VAR_g[1], 0, 0, 0, self.IMUgreenRob.VAR_g[2]]
        ##t1 = time()
        ##self.publisher.talkerSimulator(self.IMUgreenRob.nowTime, self.greenRob.quaternion, self.IMUgreenRob.GyroSensorSimulate, covarianceGyro, self.IMUgreenRob.accelSensorSimulate, covarianceAccel, position)
        print(self.IMUgreenRob.nowTime - self.IMUgreenRob.lastTime)
        if status == "Andando":
            self.zeroVelocity.append(0)
        elif status == "Parado":
            self.zeroVelocity.append(1)
        elif status == "Fim" and self.dataSave == False:
            self.dataSave=True
            self.zeroVelocity.append(1)
            df = pd.DataFrame({'xPos': self.xPos,
                               'yPos': self.yPos,
                               'zPos': self.zPos,
                               'vx': self.vx,
                               'vy': self.vy,
                               'vz': self.vz,
                               'wx': self.wx,
                               'wy': self.wy,
                               'wz': self.wz,
                               'qw': self.qw,
                               'qx': self.qx,
                               'qy': self.qy,
                               'qz': self.qz,
                               'zeroVelocity': self.zeroVelocity,
                               'time': self.time
                             })
            df.to_csv('real.csv')
            print(len(self.sensorAccelX))
            print(len(self.sensorAccelY))
            print(len(self.sensorAccelZ))
            print(len(self.sensorGyroX))
            print(len(self.sensorGyroY))
            print(len(self.sensorGyroZ))
            print(len(self.time))
            df2 = pd.DataFrame({'time': self.time,
                                'qw': self.qw,
                                'qx': self.qx,
                                'qy': self.qy,
                                'qz': self.qz,
                                'accelX': self.sensorAccelX,
                                'accelY': self.sensorAccelY,
                                'accelZ': self.sensorAccelZ,
                                'GyroX': self.sensorGyroX,
                                'GyroY': self.sensorGyroY,
                                'GyroZ': self.sensorGyroZ,
                                'accelXn': self.sensorAccelNoiseX,
                                'accelYn': self.sensorAccelNoiseY,
                                'accelZn': self.sensorAccelNoiseZ,
                                'GyroXn': self.sensorGyroNoiseX,
                                'GyroYn': self.sensorGyroNoiseY,
                                'GyroZn': self.sensorGyroNoiseZ,
                              })
            df2.to_csv('sensorCoppelia.csv')
        ##t2 = time()
        #print("Tempo de publisher: ",t2-t1)
