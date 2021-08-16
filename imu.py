from numpy import arctan2,pi,sqrt,cos,sin,array,matmul,amin,where,zeros,delete,append,    random, sort, float32, rad2deg, deg2rad
import matplotlib.pyplot as plt
import sim,simConst

#? Operation modes for API
opmblock=sim.simx_opmode_blocking
opmstream=sim.simx_opmode_streaming
opmbuffer=sim.simx_opmode_buffer
opmoneshot=sim.simx_opmode_oneshot

class IMU:
    def __init__(self, robot, sufix):
        # Variaveis gerais
        self.sufix = sufix                  # Sufixo utilizado no simulador (Vazio, '#0' ou '#1')
        self.printRand = False              # TEMP
        self.simStream=False                # Flag de stream simulador
        self.robot = robot                  # Robo que será simulado a IMU
        self.lastTime = 0.0                 # Tempo da ultima iteração no simulador
        self.nowTime = 0.0                  # Tempo da iteração atual
        self.dt = 0                         # Intervalo de tempo de iteração
        self.lastLinearVelocity = [0, 0, 0] # Velocidades lineares da antiga iteração (Ref: Mundo)
        self.linearVelocity = [0, 0, 0]     # Velocidades lineares da iteração atual (Ref: Mundo)
        self.angularVelocity = [0, 0, 0]    # Velocidades angulares (Ref: Corpo)
        self.acceleration = [0, 0, 0]       # Aceleração calculada numéricamente
        self.alpha=0                        # Angulo de rotação em relação a X
        self.beta=0                         # Angulo de rotação em relação a Y
        self.gamma=0                        # Angulo de rotação em relação a Z
        self.streamAngle = False
        self.simulateTime = 600              # Tempo de simulação para plotar os dados

        # Giroscópio
        self.VAR_g = [0.1105 , 0.0760 , 0.1170 ]       # Valores [x, y, z] de VAR_g
        self.VAR_bg = [12.6653 , 14.7800 , 0.5600 ]    # Valores [x, y, z] de VAR_bg
        self.TAU_g = [1759.07, 1621.95, 1093.75]                                                # Valores [x, y, z] de tau_g

        self.b_g_last = [0, 0, 0]           # Valores [x, y, z] do ultimo calculo de b_g

        self.gyroData = [[], [], []]        # Variavel para plotar os dados do giroscópio em plotSensors

        self.GyroSensor = [0, 0, 0]         # Variavel que armazena a leitura atual do Giroscópio
        self.GyroSensorSimulate = [0, 0, 0]
        self.GyroSensorFilterSimulate = [0, 0, 0]

        self.gyro = [[], [], []]            # Armazenamento de valores [x, y, z] obtidos no giroscópio
        self.gyroFiltered = [[], [], []]    # Armazenamento de valores [x, y, z] obtidos na simulação e filtro passa-baixa
        self.simGyro = [[], [], []]         # Armazenamento de valores [x, y, z] simulados do giroscópio com ruído
        self.simGyroFiltered = [[], [], []] # Armazenamento de valores [x, y, z] simulados do giroscópio com ruído e filtro passa-baixa
        self.timeGyro = []

        self.startLowPassFilter = [False, False, False]

        self.conterGyro = 0                 # TEMP Contador para plotar dados do giroscópio
        self.plotData = False               #      flag para plotar os dados na função de simulação

        # Acelerometro
        self.VAR_a = [0.0040, 0.0036, 0.0024]     # Valores [x, y, z] de VAR_a
        self.VAR_ba = [0.0003, 0.0100, 0.0050]    # Valores [x, y, z] de VAR_ba
        self.TAU_a = [242.24, 4132.23, 4444.44]     # Valores [x, y, z] de tau_a

        self.b_a_last = [0, 0, 0]           # Valores [x, y, z] do ultimo calculo de b_a

        self.accelData = [[], [], []]       # Variavel para plotar os dados do acelerometro em plotSensors

        self.accelSensor = [0, 0, 0]          # Variavel que armazena a leitura atual do Acelerometro
        self.accelSensorSimulate = [0, 0, 0]

        self.simAccel = [[], [], []]        # Valores [x, y, z] simulados do acelerometro com ruído
        self.accel = [[], [], []]           # Valores [x, y, z] obtidos na simulação
        self.timeAccel = []

        self.conterAccel = 0                # TEMP Contador para plotar dados do acelerometro
        self.plotDataAcelerometer= False    #      flag para plotar os dados na função de simulação

        # Leitura dos angulos de euler

        self.dataGamma = []
        self.dataGammaFilter = []



    def getClientID(self,clientID):
        '''
            Adquire o clientID da simulação para funções da API
        '''
        self.clientID = clientID


    def getVelocity(self):
        '''
            Função que obtem a velocidade linear e angular da simulação
            Velocidade Linear: Referencial no Mundo
            Velocidade Angular: Referencial no Corpo
        '''
        self.lastLinearVelocity = self.linearVelocity           # Atualiza a variavel de memória de velocidades

        # Medindo a velocidade do robô na simulação
        if(not self.simStream):
            self.resVL, self.linearVelocity, self.angularVelocity=sim.simxGetObjectVelocity(self.clientID, self.robot.soccerRob, opmstream)
            self.resAccData, self.accel_data = sim.simxGetFloatSignal(self.clientID, 'accelerometerX', opmstream)
            self.simStream = True
        else:
            self.resVL, self.linearVelocity, self.angularVelocity=sim.simxGetObjectVelocity(self.clientID, self.robot.soccerRob, opmbuffer)
            self.resAccData, self.accel_data = sim.simxGetFloatSignal(self.clientID, 'accelerometerX', opmstream)
            self.simStream = True
        if(self.resVL != 0 and self.resVL != 1):
            print('Erro durante a obtenção das velocidades \nTurning off the simulation')
            sim.simxFinish(self.clientID)
            exit()

        self.linearVelocity[0] = self.linearVelocity[0] * 100   # Conversão de m/s pra cm/s
        self.linearVelocity[1] = self.linearVelocity[1] * 100
        self.linearVelocity[2] = self.linearVelocity[2] * 100

        '''print("--------")
        modulo = sqrt(self.linearVelocity[0]**2 + self.linearVelocity[1]**2 + self.linearVelocity[2]**2)
        print("Modulo: " + str(modulo))
        print("Vx: " + str(self.linearVelocity[0])) #Velocidades em relação ao mundo
        print("Vy: " + str(self.linearVelocity[1]))
        print("Vz: " + str(self.linearVelocity[2]))
        print("Angular em X: " + str(self.angularVelocity[0])) #Velocidades em relação ao corpo
        print("Angular em Y: " + str(self.angularVelocity[1]))
        print("Angular em Z: " + str(self.angularVelocity[2]))'''


    def getAccelerometer(self):
        '''
            Função que obtem os dados do sensor Acelerometro no robô
            Obs: int_aux, string_aux e buffer_aux são variaveis temporarias
                 apenas para o funcionamento do comando CallScriptFunction
        '''
        emptyBuff = bytearray()
        self.resAccSensor, int_aux, self.accelSensor, string_aux, buffer_aux = sim.simxCallScriptFunction(self.clientID, 'Accelerometer' + self.sufix, sim.sim_scripttype_childscript,'getAccel', [], [], [], emptyBuff, sim.simx_opmode_blocking)
        #print(self.accelSensor)


    def getGyroscope(self):
        '''
            Função que obtem os dados do sensor Giroscópio no robô e o tempo de simulação
            Obs: int_aux, string_aux e buffer_aux são variaveis temporarias
                 apenas para o funcionamento do comando CallScriptFunction
        '''
        emptyBuff = bytearray()
        self.resGyroSensor, int_aux, self.dataSensor, string_aux, buffer_aux = sim.simxCallScriptFunction(self.clientID, 'GyroSensor' + self.sufix, sim.sim_scripttype_childscript,'getGyro', [], [], [], emptyBuff, sim.simx_opmode_blocking)

        self.GyroSensor[0] = self.dataSensor[0]     # Dados [x, y, z] do giroscópio
        self.GyroSensor[1] = self.dataSensor[1]
        self.GyroSensor[2] = self.dataSensor[2]

        self.nowTime = self.dataSensor[3]           # Tempo atual de simulação
        self.dt = (self.nowTime-self.lastTime)      # Intervalo de tempo entre uma execução e outra
        self.lastTime = self.nowTime                # Atualizando tempo
        self.gamma = self.dataSensor[4]
        self.gammaFilter = self.dataSensor[5]


    def getDataSensors(self):
        '''
            Função que obtem os dados do sensores Acelerometro e Giroscópio no
            robô móvel de uma vez só
            Obs: int_aux, string_aux e buffer_aux são variaveis temporarias
                 apenas para o funcionamento do comando CallScriptFunction
        '''


        emptyBuff = bytearray()                 # bytearray para captura de dados (auxiliar apenas)
        self.resDataSensors, int_aux, data, string_aux, buffer_aux = sim.simxCallScriptFunction(self.clientID, 'soccerRob_dyn#0', sim.sim_scripttype_childscript,'testeDados', [], [], [], emptyBuff, sim.simx_opmode_blocking)
        if self.resDataSensors != 0:
            print("Há algo errado com a captura de dados dos sensores!")


        self.accelSensor[0] = data[0]           # Dados [x, y, z] do acelerometro
        self.accelSensor[1] = data[1]
        self.accelSensor[2] = data[2]
        self.GyroSensor[0] = data[3]            # Dados [x, y, z] do giroscópio
        self.GyroSensor[1] = data[4]
        self.GyroSensor[2] = data[5]


        self.nowTime = data[6]                  # Tempo de simulação
        self.dt = (self.nowTime-self.lastTime)  # Intervalo de tempo
        self.lastTime = self.nowTime            # Atualização do tempo anterior


    def updateAngles(self):
        '''
            Função que obtem os angulos alpha, beta e gamma do robô na simulação,
            referentes ao sistema de coordenadas global
        '''
        # Bloco para adquirir os angulos do robô na simulação
        if not self.streamAngle:
            resOri, angles = sim.simxGetObjectOrientation(self.clientID, self.robot.soccerRob, -1, opmstream)
            self.streamAngle = True
        else:
            resOri, angles = sim.simxGetObjectOrientation(self.clientID, self.robot.soccerRob, -1, opmbuffer)

        if(resOri != 0):
            print("Problemas ao adquirir os angulos do robô")

        # Atribuindo cada angulo para as variaveis
        self.alpha = angles[0]
        self.beta = angles[1]
        self.gamma = angles[2]


    def lowPassFilter(self, a, xi, x):
        return xi*a + (1-a)*x


    def gyroSimulate0(self):
        '''
            Função para simular o giroscópio (Sem o giroscópio com o ruído branco
            e com o bias, a partir das constantes da IMU
        '''

        self.w_bg = [0, 0, 0]           # Inicialização das variaveis w_bg, bPonto_g e b_g
        self.w_g = [0, 0, 0]
        self.bPonto_g = [0, 0, 0]
        self.b_g = [0, 0, 0]

        # Calcula o ruído e o bias para os 3 eixos do giroscópio
        for i in range(3):
            self.w_bg[i] = self.VAR_bg[i] * random.randn()
            self.w_g[i] = self.VAR_g[i] * random.randn()
            self.bPonto_g[i] = - (1/self.TAU_g[i]) * self.w_bg[i]
            self.b_g[i] = self.b_g_last[i] + self.dt*self.bPonto_g[i]

        # Armazenamento dos dados do giroscópio para plotar
        if self.conterGyro < 500:

            print(self.conterGyro)

            for i in range(3):
                self.gyro[i].append(self.angularVelocity[i])                # Dados diretos da simulação

                # IF para filtrar os dados vindos da simulação
                if not self.startLowPassFilter[i]:                          # Se a Filtragem não iniciou
                    self.gyroFiltered[i].append(self.angularVelocity[i])    # Adicionar o primeiro elemento
                    self.startLowPassFilter[i] = True                       # Iniciar a Filtragem
                else:
                    dataFilter = self.lowPassFilter(0.9, self.gyroFiltered[i][-1], self.angularVelocity[i])     # Filtrar o valor da simulação
                    self.gyroFiltered[i].append(dataFilter)                                                     # Adicionar valor filtrado

                wg = self.angularVelocity[i] + self.w_g[i] + self.b_g[i]    # Adicionando ruído branco e bias no dado do sensor
                #print("w_g: " + str(self.w_g[2]))
                self.simGyro[i].append(wg)                                  # Adicionando dado do sensor simulado
                wg_f = self.gyroFiltered[i][-1] + self.w_g[i] + self.b_g[i]
                self.simGyroFiltered[i].append(wg_f)                        # Adicionando ruído e bias nos valores simulados

            self.conterGyro = self.conterGyro + 1

        # Plot dos dados calculados acima
        if self.conterGyro == 500:
            plt.figure(figsize=(16,8))
            #plt.plot(self.gyro[0], label = 'Valores X sem ruído')
            #plt.plot(self.gyro[1], label = 'Valores Y sem ruído')
            plt.plot(self.gyro[2], label = 'Valores Z sem ruído')
            plt.plot(self.gyroFiltered[2], label = 'Valores Z sem ruído filtro passa-baixa')
            #plt.plot(self.simGyro[0], label = 'Valores X com ruído')
            #plt.plot(self.simGyro[1], label = 'Valores Y com ruído')
            #plt.plot(self.simGyro[2], label = 'Valores Z com ruído')
            plt.plot(self.simGyroFiltered[2], label = 'Valores X com ruído')
            plt.legend()
            plt.show()
            self.conterGyro = self.conterGyro + 1 # TESTE SEM GIROSCÓPIO - RESULTADO IGUAL AO GIROSCÓPIO


    def gyroSimulate(self, flagPlot = False):
        '''
            Função para simular o giroscópio com o ruído branco e com o bias, a
            partir das constantes da IMU
        '''

        self.w_bg = [0, 0, 0]                                                   # Inicialização das variaveis w_bg, w_g, bPonto_g e b_g
        self.w_g = [0, 0, 0]
        self.bPonto_g = [0, 0, 0]
        self.b_g = [0, 0, 0]

        print(self.dt)
        # Calcula o ruído e o bias para os 3 eixos do giroscópio
        for i in range(3):
            self.w_bg[i] = sqrt(self.VAR_bg[i]) * random.randn()
            self.w_g[i] = sqrt(self.VAR_g[i]) * random.randn()
            #self.bPonto_g[i] = - (1/self.TAU_g[i]) * self.w_bg[i]
            #self.b_g[i] = self.b_g_last[i] + self.dt*self.bPonto_g[i]
            self.b_g[i] = self.dt*( -1/self.TAU_g[i] * self.b_g_last[i]  + self.w_bg[i] ) + self.b_g_last[i]
            self.b_g_last[i] = self.b_g[i]
            #print(self.b_g[i])


            #Incorpora o ruído nos dados do sensor
            self.GyroSensorSimulate[i] = deg2rad(rad2deg(self.GyroSensor[i]) + self.w_g[i] + self.b_g[i])        # Adicionando ruído branco e bias no dado do sensor
            #print("self.accelSensorSimulate[i]: " + str(self.w_a[2]))

        #Plota os gráficos
        if flagPlot:                                                            # Verifica a flag parametro da função
            self.timeGyro.append(self.nowTime)                                  # Salva o tempo de leitura

            # DESCOMENTAR SE OLHAR O COMPORTAMENTO DOS ANGULOS

            #self.dataGamma.append(self.gamma)
            #self.dataGammaFilter.append(self.gammaFilter)

            print(self.nowTime)
            for i in range(3):
                self.gyro[i].append(self.GyroSensor[i])                         # Salvando dados do Giroscópio
                self.simGyro[i].append(self.GyroSensorSimulate[i])

                # Filtrando dados do giroscópio
                if not self.startLowPassFilter[i]:                              # Se a Filtragem não iniciou
                    self.gyroFiltered[i].append(self.GyroSensor[i])             # Adicionar o primeiro elemento
                    self.startLowPassFilter[i] = True                           # Iniciar a Filtragem
                else:
                    dataFilter = self.lowPassFilter(0.9, self.gyroFiltered[i][-1], self.GyroSensor[i])          # Filtrar o valor da simulação
                    self.gyroFiltered[i].append(dataFilter)                                                     # Adicionar valor filtrado
                self.GyroSensorFilterSimulate[i] = self.gyroFiltered[i][-1] + self.w_g[i] + self.b_g[i]         # Adicionando ruído e bias nos valores simulados
                self.simGyroFiltered[i].append(self.GyroSensorFilterSimulate[i])                                # Salvando dados filtrados e simulados para plot

            if (self.nowTime > self.simulateTime) and (not self.plotData):
                plt.figure(figsize=(16,8))
                plt.plot(self.timeGyro, self.gyro[0], label = 'Valores X sem ruído')
                plt.plot(self.timeGyro, self.gyro[1], label = 'Valores Y sem ruído')
                plt.plot(self.timeGyro, self.gyro[2], '.', label = 'Valores Z sem ruído')
                plt.title("Giroscópio sem ruído")
                plt.legend()
                plt.show(block=False)

                plt.figure(figsize=(16,8))
                plt.plot(self.timeGyro, self.simGyro[0], label = 'Valores X com ruído')
                plt.plot(self.timeGyro, self.simGyro[1], label = 'Valores Y com ruído')
                plt.plot(self.timeGyro, self.simGyro[2], label = 'Valores Z com ruído')

                plt.title("Giroscópio")
                plt.legend()
                plt.show(block=False)


                #plt.figure(figsize=(16,8))
                #plt.plot(self.timeGyro, self.dataGammaFilter, '.', label = 'Valores gamma Filtrados')
                #plt.plot(self.timeGyro, self.dataGamma, '.', label = 'Valores gamma')
                #plt.title("Euler angles")
                #plt.legend()

                plt.figure(figsize=(16,8))
                plt.plot(self.timeGyro, self.simGyroFiltered[0], label = 'Valores Z com ruído filtro passa-baixa')
                plt.plot(self.timeGyro, self.simGyroFiltered[1], label = 'Valores Z com ruído filtro passa-baixa')
                plt.plot(self.timeGyro, self.simGyroFiltered[2], label = 'Valores Z com ruído filtro passa-baixa')
                plt.title("Giroscópio com filtro")
                plt.legend()
                plt.show()
                self.plotData = True

            self.conterGyro = self.conterGyro + 1


    def accelSimulate(self, flagPlot = False):
        '''
            Função para simular o acelerometro com o ruído branco e com o bias, a
            partir das constantes da IMU
        '''

        self.w_ba = [0, 0, 0]                                                   # Inicialização das variaveis w_ba, w_a, bPonto_a e b_a [x, y, z]
        self.w_a = [0, 0, 0]
        self.bPonto_a = [0, 0, 0]
        self.b_a = [0, 0, 0]

        # Calcula o ruído e o bias para os 3 eixos do acelerometro
        for i in range(3):
            self.w_ba[i] = sqrt(self.VAR_ba[i]) * random.randn()
            self.w_a[i] = sqrt(self.VAR_a[i]) * random.randn()
            #self.bPonto_a[i] = - (1/self.TAU_a[i]) * self.w_ba[i]
            #self.b_a[i] = self.b_a_last[i] + self.dt*self.bPonto_a[i]
            self.b_a[i] = self.dt*( -1/self.TAU_a[i] * self.b_a_last[i]  + self.w_ba[i] ) + self.b_a_last[i]
            self.b_a_last[i] = self.b_a[i]
            #print(self.b_a[i])

            #Incorpora o ruído nos dados do sensor
            self.accelSensorSimulate[i] = self.accelSensor[i] + self.w_a[i] + self.b_a[i]        # Adicionando ruído branco e bias no dado do sensor
            #print("self.accelSensorSimulate[i]: " + str(self.w_a[2]))

        #Plota os gráficos
        if flagPlot:                                                            # Verifica a flag parametro da função
            self.timeAccel.append(self.nowTime)                                 # Salva o tempo de leitura
            #print(self.nowTime)
            for i in range(3):
                self.accel[i].append(self.accelSensor[i])                       # Salvando dados do Acelerometro
                self.simAccel[i].append(self.accelSensorSimulate[i])            # Salvando dados do sensor simulado

            if (self.nowTime > self.simulateTime) and (not self.plotDataAcelerometer):
                plt.figure(figsize=(16,8))
                plt.plot(self.timeAccel, self.accel[0], label = 'Valores X sem ruído')
                plt.plot(self.timeAccel, self.accel[1], label = 'Valores Y sem ruído')
                plt.plot(self.timeAccel, self.accel[2], label = 'Valores Z sem ruído')
                plt.plot(self.timeAccel, self.simAccel[0], label = 'Valores X com ruído')
                plt.plot(self.timeAccel, self.simAccel[1], label = 'Valores Y com ruído')
                plt.plot(self.timeAccel, self.simAccel[2], label = 'Valores Z com ruído')
                plt.title("Acelerometro")
                plt.legend()
                plt.show(block=False)
                self.plotDataAcelerometer = True

            self.conterAccel = self.conterAccel + 1


    def simulateIMU(self):
        '''
            Função que reune todas as chamadas de informação na IMU
        '''
        #Obs: Se quiser plotar o angulo de Euler, usar a getGyroscope


        self.updateAngles()                 # Funções extras
        self.getVelocity()


        #self.getAccelerometer()
        #self.getGyroscope()
        self.getDataSensors()
        self.accelSimulate(flagPlot = False)
        self.gyroSimulate(flagPlot = False)
        #self.lagrangeInterpolation()


    # FUNÇÕES TEMPORÁRIAS


    def plotSensors(self):
        if self.conterAccel < 500:
            for i in range(3):
                self.accelData[i].append(self.accelSensor[i])
                self.gyroData[i].append(self.GyroSensor[i])
            self.conterAccel = self.conterAccel + 1
            print(self.conterAccel)
        elif self.conterAccel == 500:
            plt.figure(figsize=(16,8))
            plt.plot(self.accelData[0], label = 'Aceleração em X')
            plt.plot(self.accelData[1], label = 'Aceleração em Y')
            plt.plot(self.accelData[2], label = 'Aceleração em Z')
            plt.legend()

            plt.figure(figsize=(16,8))
            plt.plot(self.gyroData[0], label = 'Velocidade Angular em X')
            plt.plot(self.gyroData[1], label = 'Velocidade Angular em Y')
            plt.plot(self.gyroData[2], label = 'Velocidade Angular em Z')
            plt.legend()
            plt.show()
            self.conterAccel = self.conterAccel + 1


    def plotRand(self):
        '''
            Função temporária para testar a função numpy.randn()
        '''
        if not self.printRand:
            teste1 = random.randn(1000)
            teste1 = sort(teste1)
            plt.figure(figsize=(16,8))
            plt.hist(teste1, bins = 10, label='Ganho')

            teste2 = []
            for i in range(1000):
                teste2.append(random.randn())
            plt.show(block=False)
            plt.figure(figsize=(16,8))
            plt.hist(teste2, bins = 10, label='Ganho')
            plt.show()

            self.printRand = True
        # RESULTADO: AS DUAS OPÇÕES RESULTAM EM DISTRIBUIÇÕES NORMAIS
