import numpy as np

class CF:
    def __init__(self):
        self.Kp = 0.5                  # Ganho proporcional do filtro
        self.Ki = 3                    # Ganho integrativo do filtro

        self.phi_fe = 0                 # Angulo do Filtro Complementar Explícito
        self.phi_v = 0                  # Angulo da Visão Computacional
        self.phi_f = 0                  # Angulo final do Filtro Complementar

        self.e = 0                      # Erro do Filtro Complementar
        self.somaErro = 0               # Erro acumulado do Filtro Complementar

        self.start = False              # Flag para o start do Filtro


    def calcDelta(self, dt):
        '''
            Função para calcular a componente Delta de correção no Filtro
        '''
        self.e =  -1 * np.arctan2(np.sin(self.phi_v), np.cos(self.phi_v)) + np.arctan2(np.sin(self.phi_f), np.cos(self.phi_f))
        self.e = np.arctan2(np.sin(self.e), np.cos(self.e))     # Erro entre o angulo atual e o da visão
        #self.e = self.phi_v - self.phi_f                       # Erro entre o angulo atual e o da visão
        print("Visao: ", self.phi_v)
        #print("e: ", self.e)
        self.somaErro += self.e*dt                              # Acumulando o erro para o controlador PI
        self.delta = self.Kp*self.e + self.Ki*self.somaErro     # Calculo de Delta
        #print("delta: ", self.delta)


    def calcPhiF(self):
        '''
            Função para calcular a orientação final do filtro
        '''
        self.phi_f = self.phi_fe - self.delta


    def main(self, phi_fe, phi_v, dt, timeImu, timeVision):
        '''
            Função principal do filtro. Coordena todos os calculos relacionados a ele
        '''
        self.phi_fe = phi_fe            # Salva o dado do ECF
        self.phi_v = phi_v              # Salva o dado da visão

        if not self.start:              # Se o filtro não iniciou:
            self.phi_f = phi_v          # Valor inicial para o EC como o valor de ECF
            self.start = True
        else:                           # Se não
            self.calcDelta(dt)          # Calcula a componente delta
            self.calcPhiF()             # Calcula a orientação final
