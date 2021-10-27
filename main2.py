import sim, simConst
from strategy import StrategyTesting
import time


#TODO #1 Transformar isso em função - Observar a possibilidade em fazer isso na StrategyTesting
#Conecção v-rep, caso não estiver aberto o programa encerra
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1',20001,True,True,5000,5) # Connect to V-REP
game = StrategyTesting()
#draw = DrawRedDragons()


#Se houve a comunicação com sucesso ativa a flagComunicar
if clientID!=-1:
    game.simConnect(clientID)
    #draw.simConnect(clientID)
    print ('Connected to remote API server')


#Se houve erro na comunicação não ativa
else:
    print('Connection not successful')


while(1):
    #t1 = time.time()
    game.play()
    #t2 = time.time()
    #print("Tempo de simulação: " + str(t2-t1))


    #draw.play()
