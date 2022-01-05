import numpy as np

class AngularVelocity:
    def __init__(self):
        self.dt = 0                                 # Intervalo de tempo de medição
        self.lastOrientation = 0                    # Orientação anterior
        self.orientation = 0                        # Orientação atual

        self.start = False                          # Flag de inicio do calculo de velocidade angular

        self.angularVelocity = 0                    # Velocidade Angular medida

        self.lastAngVel = 0                         # Armazena os ultimos dados de aceleração
        self.saveAngVel = 0                         # Velocidade angular sem filtro passa-baixa salva
        self.anVelFiltered = 0                      # Velocidade angular filtrada do filtro passa-baixa
        self.startFilter = False                    # Flag que indica o inicio do filtro passa-baixa
        self.filtered = 0


    def lowPassFilter(self, a, xi, x):
        '''
            Função de Filtro Passa-Baixa
        '''
        return xi*a + (1-a)*x

    def velocityFilter(self):
        '''
            Função de filtragem dos dados da aceleração para utilização, caso
            necessário, no algoritmo de velocidade zero.
        '''
        if not self.startFilter:                                                # Se o filtro não iniciou ainda
            self.lastAngVel = self.angularVelocity                              # Salva a ultima aceleração medida para cada eixo
            self.filtered = self.angularVelocity                                # Define a aceleração inicial filtrada como a própria lida no sensor
            self.startFilter = True                                             # Muda a flag pois o filtro foi iniciado

        else:                                                                   # Se o filtro iniciou
            self.filtered = self.lowPassFilter(0.9, self.filtered, self.angularVelocity) # Filtra os dados da aceleração

    def main(self, dt, orientation):

        self.orientation = orientation                                          # Orientação recebida do filtro
        self.dt = dt                                                            # Intervalo de tempo atual

        if not self.start:                                                      # Se o filtro não iniciou
            self.lastOrientation = orientation                                  # Salva a ultima orientação como a atual
            self.start = True                                                   # Inicia o calculo da velocidade
        else:
            self.angularVelocity = (self.orientation - self.lastOrientation)/dt # Calculo da derivada do angulo para velocidade angular
            print("Diferenca de angulo: ", self.orientation)
            print("dt: ", dt)
            print("Velocidade angular: ", self.angularVelocity)
            self.velocityFilter()                                               # Filtro passa baixa na velocidade calculada
            self.lastOrientation = orientation                                  # Salva a orientação anterior
