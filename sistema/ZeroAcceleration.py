from numpy import array, sqrt, rad2deg
import matplotlib.pyplot as plt

class ZeroAcceleration:
    def __init__(self, imu, g):
        self.imu = imu

        self.bu = 0.35                                                          # Treshold para o acelerometro sem Gravidade
        self.bg = 0.15                                                          # Treshold para o giroscópio
        self.bf = 0.25                                                          # Treshold para o acelerometro sem Gravidade com Filtro Passa-Baixa

        self.mi = 0                                                             # Módulo do vetor aceleração subtraido do módulo da gravidade
        self.u = 0                                                              # Modulo do vetor velocidade angular
        self.mi_f = 0                                                           # Módulo do vetor aceleração filtrado subtraido do módulo da gravidade

        self.flag_u = False                                                     # Flag para os dados do acelerômetro
        self.flag_g = False                                                     # Flag para os dados do giroscópio
        self.flag_f = False                                                     # Flag para os dados do acelerometro filtrados

        self.lastAccel = [0, 0, 0]                                              # Variaveis do filtro passa-baixa
        self.filtered = [0, 0, 0]
        self.startFilter = False

        self.g = [0, 0, 0]                                                      # Declaração inicial de g
        self.g[0] = g[0]
        self.g[1] = g[1]
        self.g[2] = g[2]
        self.updateG = False                                                    # Flag que indica a atualização do vetor gravidade

        self.ya = 0                                                             # Modulo do vetor aceleracao
        self.ya_f = 0                                                           # Modulo do vetor aceleracao filtrado
        self.ga = 0                                                             # Modulo do vetor gravidade

        self.notAccel = False                                                   # Flag que indica que o robô não está acelerando


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
                self.filtered[i] = self.lowPassFilter(0.6, self.filtered[i], self.imu.accel[i]) # Filtra os dados da aceleração para cada eixo

    def main(self, zeroVelocity, g):
        '''
            Função main: Função onde ocorre a chamada de todas as funções
            auxiliares para a detecção da Aceleração Zero
        '''
        self.ya = sqrt(self.imu.accel[0]**2 + self.imu.accel[1]**2 + self.imu.accel[2]**2) # Calculo de ya: Módulo do vetor aceleração do sensor
        self.ga = sqrt(self.g[0]**2 + self.g[1]**2 + self.g[2]**2)              # Calculo do módulo do vetor gravidade definido
        self.mi = self.ya - self.ga                                             # Aceleração sem gravidade em módulo

        self.updateG = True                                                     # Mudança do updateG

        if self.updateG:
            if abs(self.mi) < self.bu:                                          # Verificação do Treshold para o acelerometro sem filtro
                self.flag_u = True                                              # Se for menor, então a primeira condição é verdadeira

            else:
                self.flag_u = False

        self.u = sqrt(self.imu.gyro[0]**2 + self.imu.gyro[1]**2 + self.imu.gyro[2]**2) # Calculo de u: Módulo da velocidade angular medida no giroscópio
        if self.u < self.bg:                                                    # Verificação do Treshold para o giroscopio
            self.flag_g = True                                                  # Se for menor, então a segunda condição é verdadeira (Não usada)
        else:
            self.flag_g = False

        self.accelFilter()                                                      # Realiza a filtragem dos dados do acelerometro
        self.ya_f = sqrt(self.filtered[0]**2 + self.filtered[1]**2 + self.filtered[2]**2)   # Calcula o módulo desse vetor aceleração filtrado
        self.mi_f = self.ya_f - self.ga                                         # Calcula a diferença em módulo da aceleração do sensor e da gravidade

        if self.updateG:
            if abs(self.mi_f) < self.bf:                                        # Verificação do Treshold para o acelerometro filtrado
                self.flag_f = True                                              # Se for menor, então a terceira condição é verdadeira
            else:
                self.flag_f = False

        if self.flag_u and self.flag_f:                                         # Verificação das duas condições do acelerometro sejam verdadeiras
            self.notAccel = True                                                # Caso verdadeira ambas, o robô não está acelerando
        else:
            self.notAccel = False                                               # Caso contrário, o robô está acelerando
