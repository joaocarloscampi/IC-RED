import numpy as np
from tf.transformations import quaternion_multiply, euler_from_quaternion
from scipy.linalg import expm

class ECF:
    def __init__(self):
        self.v_ = [0, 0, 0]             # Versor da gravidade
        self.vHat = [0, 0, 0]           # Vetor obtido pela matriz de rotação dos quatérnios do eixo Z
        self.start = False              # Flag que indica o inicio do Filtro
        self.qHat = [0, 0, 0, 0]        # Quaternio de saida do Filtro
        self.delta = 0                  # Parametro delta controlado via erro pelo Filtro
        self.e = 0                      # Erro entre v_ e vHat
        self.somaErro = 0               # Erro acumulado
        self.euler = [0, 0, 0]          # Angulos de euler convertidos do quatérnio

        self.Kp = 10                   # Constante de ganho proporcional
        self.Ki = 10                   # Constante de ganho integrativo

    def calcV_(self, g):
        '''
            Função para calcular o parâmetro v_ para toda interação do filtro
        '''
        module_g = np.sqrt(g[0]**2 + g[1]**2 + g[2]**2)                                         # Módulo do vetor gravidade
        self.v_ = np.array([g[0]/module_g, g[1]/module_g, g[2]/module_g], dtype = np.float64)   # Calculo de v_

    def calcVHat(self):
        '''
            Função para calcular o parâmetro v_ para toda interação do filtro
        '''
        # Arrumando a ordem [x, y, z, w] da biblioteca tf.transformations para [w, x, y, z]
        qTeste = [0, 0, 0, 0]
        qTeste[0], qTeste[1], qTeste[2], qTeste[3] = self.qHat[3], self.qHat[0], self.qHat[1], self.qHat[2]

        # Calculo do parâmetro v^ (vHat)
        self.vHat = np.array([
            2*(qTeste[1]*qTeste[3] - qTeste[0]*qTeste[2]),
            2*(qTeste[2]*qTeste[3] + qTeste[0]*qTeste[1]),
            (qTeste[0]**2 - qTeste[1]**2 - qTeste[2]**2 + qTeste[3]**2)
        ], dtype=np.float64)

    def calcDelta(self, dt):
        '''
            Função para calcular a componente Delta de correção no Filtro
        '''
        self.e = np.cross(self.v_, self.vHat)                   # Produto vetorial entre v_ e vHat
        #print("e: ", self.e)
        self.somaErro += self.e*dt                              # Acumulando o erro para o controlador PI
        self.delta = self.Kp*self.e + self.Ki*self.somaErro     # Calculo de Delta
        #print("delta: ", self.delta)

    def calcQhat(self, orientation, angularVelocity, dt):
        '''
            Função que calcula e atualiza o quaternio de saída do filtro
        '''
        # Calculo do quatérnio puro qp no padrão tf.transformations [x, y, z, 0]
        qp = np.array([angularVelocity[0]+self.delta[0], angularVelocity[1]+self.delta[1], angularVelocity[2]+self.delta[2], 0], dtype = np.float64)

        # Vetor para armazenar a velocidade angular e a componente delta do filtro para correção
        vetor = [0, 0, 0]
        vetor[0], vetor[1], vetor[2] =   angularVelocity[0] + self.delta[0] + 0.0, angularVelocity[1] + self.delta[1] + 0.0, angularVelocity[2] + self.delta[2] + 0.0

        # Resolução da equação diferencial do final do filtro
        # Criação da matriz A com as componentes do vetor
        A = np.array([[ 0 , (-1)*(vetor[0]), (-1)*(vetor[1]), (-1)*(vetor[2]) ],
                      [ vetor[0] , 0 , vetor[2] , (-1)*(vetor[1]) ],
                      [ vetor[1] , (-1)*(vetor[2]) , 0 , vetor[0] ],
                      [ vetor[2] , vetor[1] , (-1)*(vetor[0]) , 0 ]])
        M = 1/2 * A * dt
        K = expm(M) # Exponencial da matriz M

        # Atualização discreta por K do quatérnio atual q
        self.qTest = np.matmul(K, np.array([[self.q[3]],
                                   [self.q[0]],
                                   [self.q[1]],
                                   [self.q[2]]]) )

        # Atualização de qHat
        self.qHat[0], self.qHat[1], self.qHat[2], self.qHat[3] = self.qTest[1][0], self.qTest[2][0], self.qTest[3][0], self.qTest[0][0]

        # Atualização do quaternio atual
        self.q[0], self.q[1], self.q[2], self.q[3] = self.qHat[0], self.qHat[1], self.qHat[2], self.qHat[3]
        self.euler[0], self.euler[1], self.euler[2] = euler_from_quaternion(self.q) # Angulos de Euler (convertidos do quatérnio)

    def main(self, orientation, g, angularVelocity, dt):
        '''
            Função principal do filtro. Coordena todos os calculos relacionados a ele
        '''

        # Se o filtro ainda não começou
        if not self.start:
            # Definição de uma orientação inicial
            self.qHat[0], self.qHat[1], self.qHat[2], self.qHat[3] = orientation[0], orientation[1], orientation[2], orientation[3]
            self.q = self.qHat
            # Trocando a flag para o inicio do Filtro
            self.start = True
        # Se o filtro já iniciou
        else:
            # Funções de calculo do filtro
            self.calcV_(g)
            self.calcVHat()
            self.calcDelta(dt)
            self.calcQhat(orientation, angularVelocity, dt)
