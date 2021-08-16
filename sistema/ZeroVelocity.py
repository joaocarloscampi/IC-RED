from numpy import array, sqrt
import matplotlib.pyplot as plt

class ZeroVelocity:

    def __init__(self, imu):

        self.imu = imu                                                          # Objeto da classe IMU que fornecerá os dados


        self.N = 5                                                              # Constante N
        self.g = 9.81                                                           # Constante aceleração da gravidade
        self.var_a = 1                                                          # Constante variância do acelerometro
        self.var_w = 1                                                          # Constante variância do giroscópio
        self.yk_a = [[], [], []]                                                # Vetor dos ultimos N dados do acelerometro
        self.yk_w = [[], [], []]                                                # Vetor dos ultimos N dados do giroscópio
        self.yn_a = [0, 0, 0]                                                   # Vetor da média das posições de yk do acelerometro
        self.yn_w = [0, 0, 0]                                                   # Vetor da média das posições de yk do giroscópio
        self.gammaLinha = 0.01                                                  # Constante de detecção de velocidade zero
        self.gamma = 0                                                          # Valores calculados pelo algoritmo para comparar com gaamaLinha

        self.gammaList = []
        self.timeList = []
        self.accelerationList = []
        self.angularList = []


    # FUNÇÕES AUXILIARES


    def getYkVector(self):
        '''
            Função que organiza as ultimas N observações do acelerômetro e do
            giroscópio nos vetores yk respectivos
        '''

        # Salvando N dados do acelerometro no vetor y_k^a
        for i in range(3):                                                      # for percorrendo os 3 eixos
            if (len(self.yk_a[i]) < self.N):                                    # Condicional para preencher os N valores
                self.yk_a[i].append(self.imu.accel[i])
            else:                                                               # Rotatividade dos valores em N posições
                aux = self.yk_a[i]                                              # Armazenamento em uma variavel auxiliar
                for j in range(1,self.N):
                    self.yk_a[i][j-1] = aux[j]                                  # Movendo os N-1 ultimos para os N-1 primeiros
                self.yk_a[i][-1] = self.imu.accel[i]                            # Atribui o N-ésimo valor

        # Salvando N dados do acelerometro no vetor y_k^w
        for i in range(3):                                                      # for percorrendo os 3 eixos
            if (len(self.yk_w[i]) < self.N):                                    # Condicional para preencher os N valores
                self.yk_w[i].append(self.imu.gyro[i])
            else:                                                               # Rotatividade dos valores em N posições
                aux = self.yk_w[i]                                              # Armazenamento em uma variavel auxiliar
                for j in range(1,self.N):
                    self.yk_w[i][j-1] = aux[j]                                  # Movendo os N-1 ultimos para os N-1 primeiros
                self.yk_w[i][-1] = self.imu.gyro[i]                             # Atribui o N-ésimo valor


    def calcYnVector(self):
        '''
            Função que calcula o vetor yn dos respectivos sensores, cujos valores
            são as médias de cada posição em yk
        '''

        # Calculando o vetor yk_a
        for i in range(3):
            sum = 0
            for j in range(self.N):
                sum = sum + self.yk_a[i][j]                                     # Soma dos N valores de cada uma dos 3 eixos
            self.yn_a[i] = sum/self.N                                           # Média dessa soma, armazenando cada eixo em yn

        for i in range(3):
            sum = 0
            for j in range(self.N):
                sum = sum + self.yk_w[i][j]                                     # Soma dos N valores de cada uma dos 3 eixos
            self.yn_w[i] = sum/self.N                                           # Média dessa soma, armazenando cada eixo em yn


    def vectorNorm(self, vector):
        '''
            Função que calcula a norma do vetor passado no parametro
        '''

        return sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)

    # FUNÇÃO PRINCIPAL

    def ZeroVelocity(self):
        '''
            Notações do artigo e projeto:
            y_k^a -> Valores do acelerometro com ruído gaussiano
            y_n^a -> Media dos n valores em y_k^a
            ||y_n^a|| -> Norma do vetor médio y_n^a
            y_k^w -> Valores do giroscópio com ruído gaussiano
        '''

        sum = 0
        self.getYkVector()

        if (len(self.yk_a[0]) == self.N and len(self.yk_w[0]) == self.N):

            self.calcYnVector()

            for i in range(self.N):
                ##print("yk_a = " + str(self.yk_a))
                ##print("yk_a[" + str(i) + "]" + " = " + str(array(self.yk_a)[:, i]))
                ##print("yn_a = " + str(self.yn_a))
                ##print("||yn_a|| = " + str(self.vectorNorm(array(self.yk_a)[:, i])))
                vector_a = array(self.yk_a)[:, i] - self.g * ( self.yn_a/self.vectorNorm(array(self.yk_a)[:, i]) )
                ##print("vector_a = " +str(vector_a))
                ##print("yk_w = " + str(self.yk_w))
                ##print("yk_w[" + str(i) + "]" + " = " + str(array(self.yk_w)[:, i]))
                ##print("------------------------")
                value = (1/self.var_a) * (self.vectorNorm(vector_a))**2 + (1/self.var_w) * (self.vectorNorm(array(self.yk_w)[:, i]))**2
                sum = sum + value

            self.timeList.append(self.imu.time)
            self.gammaList.append(self.gamma)
            self.accelerationList.append(self.imu.accel[0])
            self.angularList.append(self.imu.gyro[2])
            self.gamma = sum/self.N
            print("Gamma: " + str(self.gamma))


    def plotData(self):
        plt.figure(figsize=(16,8))
        plt.plot(self.timeList, self.gammaList, label = 'Valores gamma')
        plt.legend()
        plt.show(block = False)

        plt.figure(figsize=(16,8))
        plt.plot(self.timeList, self.accelerationList, label = 'Valores accel X')
        plt.plot(self.timeList, self.angularList, label = 'Valores gyro Z')
        plt.legend()
        plt.show()
        #print(self.timeList)
