import numpy as np
from tf.transformations import euler_from_quaternion, rotation_matrix, concatenate_matrices, quaternion_matrix

class Velocity:
    def __init__(self):

        self.Q = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]                              # Matriz de rotação dos Quatérnios
        self.imu_accel = [0, 0, 0]                                              # Aceleração medida no acelerometro
        self.imu_accel_g = [0, 0, 0]                                            # Aceleração medida no acelerometro sem a componente gravitacional
        self.imu_accel_inercial = [0, 0, 0]                                     # Aceleração medida no acelerometro no referencial inercial
        self.imu_accel_inercial_g = [0, 0, 0]                                   # Aceleração medida no acelerometro sem a componente gravitacional no referencial inercial
        self.g = [0, 0, 9.81]                                                   # Vetor gravidade inicial
        self.lastTime = 0                                                       # Ultimo tempo de leitura dos dados
        self.currentTime = 0                                                    # Tempo atual de leitura dos dados
        self.dt = 0                                                             # Intervalo de tempo da leitura atual com a leitura anterior
        self.vx = 0                                                             # Velocidade em x no referencial móvel
        self.vy = 0                                                             # Velocidade em y no referencial móvel
        self.vz = 0                                                             # Velocidade em z no referencial móvel
        self.v = 0                                                              # Velocidade em módulo
        self.v_inercial = [0, 0, 0]                                             # Velocidade no referencial inercial

        self.sumG = [0, 0, 0]                                                   # Variavel para somar os valores de g em cada eixo, para estimar sua média
        self.contSumG = 0                                                       # Conta quantas amostras serão usadas para essa estimativa
        self.calcG = True                                                       # Flag que indica que foi estimado o valor de g

        self.flagCamera = [False, False, False, False, False, False]            # Vetor de verificação da velocidade Zero pela visão
        self.lastXPos = 0                                                       # Posição X da ultima execução
        self.lastYPos = 0                                                       # Posição Y da ultima execução
        self.zeroVelocityCamera = False                                         # Flag que indica a Velocidade Zero pela visão
        self.trigger = False                                                    # Flag gatilho pra zerar a velocidade linear


    def updateCameraFlag(self, flag):
        '''
            Função utilizada para atualizar o vetor flagCamera com a ultima
            flag recebida de verifyCameraPos
        '''

        self.flagCamera.pop(0)                                                  # Retira o primeiro elemento (mais antigo) do vetor
        self.flagCamera.append(flag)                                            # Adiciona a flag mais recente no vetor

        condicional = True                                                      # Variavel auxiliar para verificar se todas as posições do vetor são True
        for flag in self.flagCamera:                                            # Verificação de todas as posições do vetor
            if not flag:
                condicional = False
                break

        if condicional:                                                         # Se todos são verdadeiros no vetor
            self.zeroVelocityCamera = True                                      # Estado de Velocidade-Zero
        else:                                                                   # Se não
            self.zeroVelocityCamera = False                                     # Não-Estado de Velocidade-Zero


    def verifyCameraPos(self, camera):
        '''
            Função utilizada para verificar se o robô móvel está em um Intervalo
            considerado parado em função de sua posição
        '''
        if camera.dataAvaliable:                                                # Se existem dados disponíveis para verificação
            print(self.flagCamera)
            print(self.lastXPos - camera.xPos)
            #print("---------------")
            if np.abs(self.lastXPos - camera.xPos) < 0.1 and np.abs(self.lastYPos - camera.yPos) < 0.1: # Se for detectado o não movimento do robô:
                self.updateCameraFlag(True)                                     # Envia True para updateCameraFlag
            else:                                                               # Se não
                self.updateCameraFlag(False)                                    # Envia False para updateCameraFlag
            self.lastXPos = camera.xPos                                         # Armazenamento das posições atuais para verificação futura
            self.lastYPos = camera.yPos


    def gravityVector2(self, zeroVelocity, imu_accel, orientation):
        '''
            Função antiga não utilizada mais
        '''
        if zeroVelocity:
            for i in range(3):
                self.sumG[i] += imu_accel[i][0]
            self.contSumG +=1
            self.calcG = False
        else:
            if not self.calcG:
                for i in range(3):
                    self.g[i] = self.sumG[i]/self.contSumG
                self.calcG = True
                self.sumG = [0, 0, 0]
                self.contSumG = 0

    def gravityVector(self, zeroVelocity, imu_accel, orientation):
        '''
            Função utilizada para estimar o vetor gravidade quando o algoritmo de
            Velocidade Zero está acionado
        '''
        if zeroVelocity:                                                        # Se o robô está em Velocidade Zero
            for i in range(3):
                self.sumG[i] += imu_accel[i]                                    # Soma os valores medidos no sensor
            self.contSumG +=1                                                   # Soma mais uma medida para estimativa
            self.calcG = False                                                  # Continua sem estimar

        else:                                                                   # Caso o algoritmo não detecte mais a Velocidade Zero
            if not self.calcG:
                for i in range(3):
                    self.g[i] = self.sumG[i]/self.contSumG                      # Calculo da média dos valores em cada eixo da gravidade
                self.calcG = True                                               # Gravidade estimada
                self.sumG = [0, 0, 0]                                           # Zera as variaveis auxiliares
                self.contSumG = 0

    def getInercialAcceleration(self, orientation, imu_accel):
        '''
            Função utilizada para transformar a aceleração medida no referencial
            móvel - do robô - no referencial inercial, utilizando sua orientação
            em quatérnio.
        '''

        self.Q = quaternion_matrix(orientation)                                 # Criação da matriz de rotação a partir da orientação q do robô
        self.Q = [ [ self.Q[0][0], self.Q[0][1], self.Q[0][2] ],                # Adequação das suas dimensões (4x4 em 3x3)
                   [ self.Q[1][0], self.Q[1][1], self.Q[1][2] ],
                   [ self.Q[2][0], self.Q[2][1], self.Q[2][2] ]]

        self.imu_accel_inercial =  np.matmul(self.Q, [[imu_accel[0]], [imu_accel[1]], [imu_accel[2]]]) # Transformação da aceleração do sensor pro referencial inercial

        self.imu_accel_inercial_g[0] = self.imu_accel_inercial[0][0] - self.g[0] # Tirando a componente da gravidade da leitura
        self.imu_accel_inercial_g[1] = self.imu_accel_inercial[1][0] - self.g[1] # em seus eixos
        self.imu_accel_inercial_g[2] = self.imu_accel_inercial[2][0] - self.g[2]

    def getInercialVelocity(self, orientation, velocity):
        '''
            Função utilizada para transformar a velocidade estimada no referencial
            móvel - do robô - para o referencial inercial, utilizando sua orientação
            em quatérnio.
        '''
        self.Q = quaternion_matrix(orientation)                                # Criação da matriz de rotação a partir da orientação q do robô
        self.Q = [ [ self.Q[0][0], self.Q[0][1], self.Q[0][2] ],               # Adequação das suas dimensões (4x4 em 3x3)
                    [ self.Q[1][0], self.Q[1][1], self.Q[1][2] ],
                    [ self.Q[2][0], self.Q[2][1], self.Q[2][2] ]]

        res =  np.matmul(self.Q, [[velocity[0]], [velocity[1]], [velocity[2]]]) # Transformação da velociadde pro referencial inercial
        self.v_inercial[0] = res[0][0]
        self.v_inercial[1] = res[1][0]
        self.v_inercial[2] = res[2][0]

    def calcLinearVelocity2(self, orientation, imu_accel, zeroVelocity, zeroAcceleration, time):
        '''
            Função antiga não utilizada mais
        '''
        self.getInercialAcceleration(orientation, imu_accel)
        self.gravityVector(zeroVelocity, self.imu_accel_inercial, orientation)

        self.currentTime = time

        '''
        if zeroVelocity:
             self.vx = 0
             self.vy = 0
             self.vz = 0
        elif(self.lastTime != 0):
            if self.dt == 0:
                v = 0
                self.lastTime = self.currentTime
                self.dt = 1
            else:
                self.dt = self.currentTime - self.lastTime
                self.vx = self.imu_accel_inercial_g[0]*self.dt + self.vx
                self.vy = self.imu_accel_inercial_g[1]*self.dt + self.vy
                self.vz = self.imu_accel_inercial_g[2]*self.dt + self.vz
        '''

        if(self.lastTime != 0):
            if self.dt == 0:
                v = 0
                self.vx = 0
                self.vy = 0
                self.vz = 0
                self.lastTime = self.currentTime
                self.dt = 1
            else:
                self.dt = self.currentTime - self.lastTime
                if zeroAcceleration:
                    #print("entrei, ", self.vz)
                    self.vx = 0*self.dt + self.vx
                    self.vy = 0*self.dt + self.vy
                    self.vz = 0*self.dt + self.vz
                else:
                    print(self.imu_accel_inercial[2])
                    print(self.g[2])
                    print(self.imu_accel_inercial_g[2])
                    print("---")
                    self.vx = self.imu_accel_inercial_g[0]*self.dt + self.vx
                    self.vy = self.imu_accel_inercial_g[1]*self.dt + self.vy
                    self.vz = self.imu_accel_inercial_g[2]*self.dt + self.vz
        #'''

        self.lastTime = self.currentTime
        self.v = np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

    def calcLinearVelocity(self, orientation, imu_accel, zeroVelocity, zeroAcceleration, time, camera):
        '''
            Função utilizada para calcular a velocidade linear do robô utiliando
            a aceleração do sensor junto com os algoritmos de detecção propostos
        '''

        #self.getInercialAcceleration(orientation, imu_accel)                    #Transformando a aceleração medida para o referencial inercial
        self.imu_accel[0], self.imu_accel[1], self.imu_accel[2] = imu_accel[0], imu_accel[1], imu_accel[2] # Salvando os dados de aceleração do sensor

        self.imu_accel_g[0] = self.imu_accel[0] - self.g[0]                     # Retirando a componente gravitacional da aceleração
        self.imu_accel_g[1] = self.imu_accel[1] - self.g[1]
        self.imu_accel_g[2] = self.imu_accel[2] - self.g[2]

        self.gravityVector(zeroVelocity, self.imu_accel, orientation)           # Estimando o próximo vetor de gravidade

        self.currentTime = time                                                 # Salvando o tempo de coleta atual de dados

        # TROCANDO VELOCIDADE ZERO PARA ACELERAÇÃO ZERO
        # CASO QUEIRA UTILIZAR A VELOIDADE ZERO, COMENTAR OS ''' ABAIXO
        '''

        # MÉTODO DE VELOCIDADE ZERO

        if zeroVelocity:                                                        # Se o algoritmo detectar que o robô está parado
             self.vx = 0                                                        # Definir todas as velocidades como 0
             self.vy = 0
             self.vz = 0

        elif(self.lastTime != 0):                                               # Se não detectar que o robô está parado
            if self.dt == 0:                                                    # Verificação de tempo igual a 0 para evitar bugs
                v = 0
                self.lastTime = self.currentTime
                self.dt = 1
            else:
                self.dt = self.currentTime - self.lastTime                      # Integração numérica da aceleração
                self.vx = self.imu_accel_g[0]*self.dt + self.vx
                self.vy = self.imu_accel_g[1]*self.dt + self.vy
                self.vz = self.imu_accel_g[2]*self.dt + self.vz
        '''

        # MÉTODO DE ACELERAÇÃO ZERO

        if(self.lastTime != 0):
            if self.dt == 0:                                                    # Verificação de tempo igual a 0 para evitar bugs
                v = 0
                self.vx = 0
                self.vy = 0
                self.vz = 0
                self.lastTime = self.currentTime
                self.dt = 1

            else:
                self.dt = self.currentTime - self.lastTime                      # Calculo do intervalo de tempo

                if zeroAcceleration:                                            # Se o algoritmo detectou que o robô não está acelerando
                    self.vx = 0*self.dt + self.vx                               # Definir a aceleração em todos os eixos como 0
                    self.vy = 0*self.dt + self.vy
                    self.vz = 0*self.dt + self.vz

                else:                                                           # Caso o algoritmo detectar aceleração
                    self.vx = self.imu_accel_g[0]*self.dt + self.vx             # Utilizar a aceleração do sensor na integração numérica
                    self.vy = self.imu_accel_g[1]*self.dt + self.vy
                    self.vz = self.imu_accel_g[2]*self.dt + self.vz
        #'''

        # UNIÃO COM A VELOCIDADE ZERO DA CAMERA

        #'''

        self.verifyCameraPos(camera)

        self.trigger = self.zeroVelocityCamera and zeroAcceleration
        if self.trigger:
            self.vx = 0
            self.vy = 0
            self.vz = 0

        self.getInercialVelocity(orientation, [self.vx, self.vy, self.vz])      # Convertendo as velocidades medidas para o referencial inercial

        self.lastTime = self.currentTime                                        # Dando update no lastTime, após todos os calculos

        self.v = np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)                  # Calculando o módulo da velocidade

        #'''
