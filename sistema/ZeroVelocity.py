from numpy import array, sqrt
import matplotlib.pyplot as plt

class ZeroVelocity:

    def __init__(self, imu):

        self.imu = imu                                                          # Objeto da classe IMU que fornecerá os dados

        self.N = 50                                                             # Constante N
        self.g = 9.81                                                           # Constante aceleração da gravidade
        self.var_a = 1                                                          # Constante variância do acelerometro
        self.var_w = 1                                                          # Constante variância do giroscópio
        self.yk_a = [[], [], []]                                                # Vetor dos ultimos N dados do acelerometro
        self.yk_w = [[], [], []]                                                # Vetor dos ultimos N dados do giroscópio
        self.yn_a = [0, 0, 0]                                                   # Vetor da média das posições de yk do acelerometro
        self.yn_w = [0, 0, 0]                                                   # Vetor da média das posições de yk do giroscópio
        self.gammaLinha = 12                                                    # Constante de detecção de velocidade zero
        self.gamma = 0                                                          # Valores calculados pelo algoritmo para comparar com gaamaLinha
        self.stopped = False                                                    # Flag que indica o móvel parado ou não

        self.lastAccel = [0, 0, 0]                                              # Armazena os ultimos dados de aceleração
        self.saveAccel = [0, 0, 0]                                              # Aceleração sem filtro passa-baixa salva
        self.filtered = [0, 0, 0]                                               # Aceleração filtrada do filtro passa-baixa
        self.startFilter = False                                                # Flag que indica o inicio do filtro passa-baixa


    # FUNÇÕES AUXILIARES

    def lowPassFilter(self, a, xi, x):
        '''
            Função de Filtro Passa-Baixa
        '''
        return xi*a + (1-a)*x

    def accelFilter(self):
        '''
            Função de filtragem dos dados da aceleração para utilização, caso
            necessário, no algoritmo de velocidade zero.
        '''
        if not self.startFilter:                                                # Se o filtro não iniciou ainda
            for i in range(3):
                self.lastAccel[i] = self.imu.accel[i]                           # Salva a ultima aceleração medida para cada eixo
                self.filtered[i] = self.imu.accel[i]                            # Define a aceleração inicial filtrada como a própria lida no sensor
            self.startFilter = True                                             # Muda a flag pois o filtro foi iniciado

        else:                                                                   # Se o filtro iniciou
            for i in range(3):
                self.filtered[i] = self.lowPassFilter(0.8, self.filtered[i], self.imu.accel[i]) # Filtra os dados da aceleração
                self.saveAccel[i] = self.imu.accel[i]                           # Salva a ultima aceleração medida para cada eixo
                self.imu.accel[i] = self.filtered[i]                            # Define a aceleração da imu como a calculada pelo filtro em cada eixo

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

    def main(self):
        '''
            Notações do artigo e projeto:
            y_k^a -> Valores do acelerometro com ruído gaussiano
            y_n^a -> Media dos n valores em y_k^a
            ||y_n^a|| -> Norma do vetor médio y_n^a
            y_k^w -> Valores do giroscópio com ruído gaussiano
        '''

        self.accelFilter()                                                      # Inicio do filtro passa-baixa para os dados do acelerometro

        sum = 0
        self.getYkVector()

        self.var_w = (self.imu.angular_velocity_covariance[0][0] + self.imu.angular_velocity_covariance[1][1] +
                      self.imu.angular_velocity_covariance[2][2])/3
        self.var_a = (self.imu.linear_acceleration_covariance[0][0] + self.imu.linear_acceleration_covariance[1][1] +
                      self.imu.linear_acceleration_covariance[2][2])/3

        #self.var_w = (9.55989356069987e-07 + 6.507957657007658e-07 + 5.603402148024383e-07)/3          # Dados de variancia medidos do celular
        #self.var_a = (0.00015325799770760437 + 0.000812165221644058 + 0.00018227341330091776)/3

        #self.var_w = 0.0031                                                    # Utilizar esses valores quando a matriz de covariancia não for passada
        #self.var_a = 0.1050

        if (len(self.yk_a[0]) == self.N and len(self.yk_w[0]) == self.N):       # Se temos dados suficientes para iniciar

            self.calcYnVector()                                                 # Calculo de Yn

            for i in range(self.N):                                             # Calculo do somatório no artigo
                vector_a = array(self.yk_a)[:, i] - self.g * ( self.yn_a/self.vectorNorm(array(self.yk_a)[:, i]) )
                value = (1/self.var_a) * (self.vectorNorm(vector_a))**2 + (1/self.var_w) * (self.vectorNorm(array(self.yk_w)[:, i]))**2
                sum = sum + value

            self.gamma = sum/self.N                                             # Calculo de gamma, o T(zn)

        if self.gamma < self.gammaLinha:                                        # Verificação do T(zn)
            self.stopped = True                                                 # Se for menor, o robô está parado
        else:
            self.stopped = False                                                # Se for maior, o robô está em movimento

        for i in range(3):                                                      # Retorno da aceleração original do sensor
            self.imu.accel[i] = self.saveAccel[i]
