from simClasses import Ball,Robot,Target
from strategy import StrategyTesting
import sim,simConst
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QDialog,QApplication,QMainWindow
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSlot,QTimer,QThread,QEventLoop
import sys
class GUIControlador(QMainWindow):
    def __init__(self):
        super(GUIControlador,self).__init__()
        loadUi('controleGUI.ui',self)
        self.check_Comunicar.stateChanged.connect(self.Comunicar)
        #Botões:(Verificando se o botão foi clicado e atribuindo uma função a ele)
        self.btn_Jogar.clicked.connect(self.Jogar)
        self.btn_Parar.clicked.connect(self.Parar)
        self.btn_Pose.clicked.connect(self.PoseInicial)
        self.btn_Penalti.clicked.connect(self.Penalti)
        self.FlagJogar=False
        self.FlagPoseInicial=False
        self.FlagComunicar=False
        self.FlagTrocouCampo=False
        self.FlagParar=False
        self.FlagComunicar=False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.uptade)
        self.timer.start(33)
        self.game=StrategyTesting()
    def Jogar(self):
        self.FlagJogar=True
        self.FlagPoseInicial=False
        self.FlagParar=False
        self.FlagPenalti=False
        sim.simxStartSimulation(self.clientID,sim.simx_opmode_oneshot)

    def Penalti(self):
        self.FlagJogar=False
        self.FlagPoseInicial=False
        self.FlagParar=False
        self.FlagPenalti=True

    def PoseInicial(self):
        self.FlagJogar=False
        self.FlagPoseInicial=True
        self.FlagParar=False
        self.FlagPenalti=False

    def Parar(self):
        self.FlagJogar=False
        self.FlagPoseInicial=False
        self.FlagParar=True
        self.FlagPenalti=False
        sim.simxStopSimulation(self.clientID,sim.simx_opmode_oneshot)
    def printVelocidades(self):
        self.LCDD_R0.display(float(self.game.greenRob.vR))
        self.LCDE_R0.display(float(self.game.greenRob.vL))
        self.LCDD_R1.display(float(self.game.redRob.vR))
        self.LCDE_R1.display(float(self.game.redRob.vL))
        self.LCDD_R2.display(float(self.game.pinkRob.vR))
        self.LCDE_R2.display(float(self.game.pinkRob.vL))

    def uptade(self):
        #Se a comunicação não foi ativada na
        if(self.FlagComunicar == False):
            self.Log.setText("Inicie a comunicação")
        if(self.FlagComunicar == True):
            if(self.FlagJogar):
                self.game.play()
                self.printVelocidades()
    def Comunicar(self):
        #Se for selecionado na interface inicia a comunição
        if(self.check_Comunicar.isChecked()):
            #Conecção v-rep, caso não estiver aberto o programa encerra
            sim.simxFinish(-1) # just in case, close all opened connections
            self.clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
            #Se houve a comunicação com sucesso ativa a flagComunicar
            if self.clientID!=-1:
                self.game.simConnect(self.clientID)
                print ('Connected to remote API server')
                self.FlagComunicar=True
            #Se houve erro na comunicação não ativa
            else:
                print('Connection not successful')
                self.FlagComunicar=False
        #Se não está selecionado desativa a comunicação
        else:
            self.FlagComunicar=False
            print("Comunicação desativada")

app = QApplication(sys.argv)
window = GUIControlador()
window.setWindowTitle('GUIsimulador')
window.show()
sys.exit(app.exec_())
