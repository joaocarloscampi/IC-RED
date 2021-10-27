from numpy import pi

class Path:
    def __init__(self, imu):
        self.mode = -2
        self.imu = imu
        self.contador = 0

    def path1(self):
        if self.mode == -2:
            self.imu.robot.simSetVel(0,0)
            self.contador +=1
            if self.contador == 200:
                self.mode +=1
            return "Parado"

        elif self.mode == -1:
            self.imu.robot.simSetVel(10,0)
            self.contador +=1
            if self.contador == 250:
                self.mode +=1
            return "Andando"

        elif self.mode == 0:
            print("0")
            if self.imu.nowTime < 10:
                self.imu.robot.simSetVel(0,0)
            else:
                self.mode += 1
            return "Parado"

        elif self.mode == 1:
            print("1")
            if self.imu.robot.xPos < 65:
                self.imu.robot.simSetVel(10,0)
            else:
                self.mode += 1
                self.time1 = self.imu.nowTime
            return "Andando"

        elif self.mode == 2:
            print("2")
            if (self.imu.nowTime - self.time1) < 2:
                self.imu.robot.simSetVel(0,0)
            else:
                self.mode += 1
            return "Parado"

        elif self.mode == 3:
            print("3")
            if self.imu.gamma < pi/2 - pi/500:
                self.imu.robot.simSetVel(0,2)
            else:
                self.mode += 1
                self.time2 = self.imu.nowTime
            return "Andando"

        elif self.mode == 4:
            print("4")
            if (self.imu.nowTime - self.time2) < 2:
                self.imu.robot.simSetVel(0,0)
            else:
                self.mode += 1
            return "Parado"

        elif self.mode == 5:
            print("5")
            if self.imu.gamma > pi/500:
                self.imu.robot.simSetVel(20, -0.5)
            else:
                self.mode += 1
            return "Andando"

        else:
            self.imu.robot.simSetVel(0,0)
            return "Fim"
