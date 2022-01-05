# IC-RED
Neste repositório se encontra o código-fonte utilizado na minha Iniciação Científica pela FAPESP em conjunto com a Equipe de Robótica Red Dragons UFSCar. Os dados do Projeto são:

- **Título:** Sistema de estimativa de velocidades
lineares de um robô móvel
-  **Processo:** 2021/02374-1
- **Período de Vigência:** 2021/02374-1
- **Orientador:** Roberto Santos Inoue

As opiniões, hipóteses e conclusões ou recomendações expressas neste material são de responsabilidade do(s) autor(es) e não necessariamente refletem a visão da FAPESP.

# Documentação
Os documentos relacionados a esta Iniciação Científica podem ser acessados a baixo
- [Projeto de Pesquisa](https://drive.google.com/file/d/1zFUJK3YkGZgJ0GjvcVxZvgeLm42RCwwX/view?usp=sharing)
- [Relatório Parcial](https://drive.google.com/file/d/1rs7VDTy2tBbDQRIxv-mZldA7rN_o_WeH/view?usp=sharing)
- Relatório Final

## Requisitos

Os requisitos para a execução deste respositório são:
- Ubuntu 20.04.2 LTS
- ROS
- Python 3.8+
- rospy
- numpy
- matplotlib
- pandas
- tikzplotlib
- tf.transformations

## Execução

Os arquivos presentes na raiz do repositório são referentes à execução da simulação do robô móvel, não sendo necessário a execução simultanea ao sistema de medição. Caso seja de interesse acessar a simulação, basta abrir o arquivo **SimGame.ttt** no software CoppeliaSim.

Os arquivos referentes ao sistema de medição se encontram na pasta *sistema*. Para a execução, serão necessários 3 terminais de comando, executados na seguinte sequência:

1. No primeiro terminal, executar o comando:

```sh
roscore
```
Esse comando inicia as operações do ROS, possibilitando a comunicação com os bags ou tópicos que serão utilizados no algorítmo

2. No segundo terminal, executar o comando:
```sh
python3 main.py topic_imu topic_camera
```
Esse comando inicia o sistema realizando a leitura dos tópicos passados por parâmetro. O padrão de mensagem para esses dois tópicos são: Imu - sensorMsg para os dados da IMU e PoseStamped - geometry_msgs para os dados da Visão Computacional.
Se for desejado utilizar os dados desse projeto, o comando necessário será:
```sh
python3 main.py /sensor /camera
```

3. No terceiro terminal, executar o comando:
```sh
rosbag play nome_do_bag
```
Esse comando inicia o arquivo bag do ROS com os dados gravados a serem utilizados nessa simulação. É possível utilizar em tempo real a comunicação ROS com os tópicos, mas no projeto foi utilizado um bag por praticidade. Para executar esses dados, o comando é:
```sh
rosbag play path1.4.bag
```

No fim do arquivo main.py, existem várias funções de plot comentadas, nas quais geram os gráficos presentes no Relatório Parcial. Caso seja desejado ver algum desses gráficos, basta descomentar as funções desejadas (estão comentadas com o que elas plotam) e utilizar na ultima função a flag blockGraph = True (False nas anteriores).
