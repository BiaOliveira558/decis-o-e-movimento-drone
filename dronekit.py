#Não esquecer: certificar-se de que o ROS e a bib dronekit (responsável por controlar o drone) estejam instalados. Tamebm precisaremos do pacote Ros cv_camera para acessar a camera (será explicado mais para a frente)
#Estamos querendo pegar as imagens capturadas pela camera do drone e reconhecer a cor visualizada.
#Como precisamos tratar as imagens capturadas podemos utilizar a bib open cv que é uma biblioteca específica para tratar/utilizar imagens para alguma finalidade, como por exemplo visão computacional
import cv2 # aqui estamos importando um pacote python do opencv, esse pacote irá nos fornecer funções e algoritmos do opencv em python
#Ao termos a imagem capturada pela câmera precisamos de alguma forma de manipula-las para extrair a cor predominante ou seja precisaremos de alguma ferramenta para fazer nossa aplicação ler a imagem
import numpy as np # a numpy é uma bib para manipular arrays, matrizes e etc que é muito util em manipulação de imagens e visão computacional
import rospy # bib python útil para comunição entre nós do ROS
import sensor_msgs.msg # pacote do ROS responsável por transmitir dados de imagens capturados por uma camera
import cv_bridge # temos as imagens da camera e temos o open cv, precisamos de algum modo conectar os dois e esse pacote servirá para isso, ela é utilizada para converter imagens de um tipo de dado do opencv para um tipo de mensagem ROS
import dronekit # bib padrao para controlar drones 
import time # bib de tempo de pyhton

#função pra mover o drone que recebe como parametro a cor 
def move_drone(color):
    # o vehicle.channels.overrides é uma maneira de controlafr os canais de movimentação do drone remotamente permitindo sua modificação, ou modificar esses valores é possivel que o drone execute movimentos automaticamente 
    #o valor 1700 indica alguma inclinação para alguma diração. Exemplo, para mover para frente temos a inclinação logitudinal "ativada" enquanto o valor 1500 indica neutro
    if color == "green":
        # Mover o drone para frente
        vehicle.channels.overrides['1'] = 1500  # Throttle : aceleração 
        vehicle.channels.overrides['2'] = 1500  # Roll: inclinação lateral
        vehicle.channels.overrides['3'] = 1700  # Pitch : inclinação longitudinal
        vehicle.channels.overrides['4'] = 1500  # Yaw: rotaçãp
    elif color == "blue":
        # Mover o drone para direita
        vehicle.channels.overrides['1'] = 1500  # Throttle
        vehicle.channels.overrides['2'] = 1700  # Roll # inclinação para a direita
        vehicle.channels.overrides['3'] = 1500  # Pitch
        vehicle.channels.overrides['4'] = 1500  # Yaw
    elif color == "pink":
        # Mover o drone para trás
        vehicle.channels.overrides['1'] = 1500  # Throttle
        vehicle.channels.overrides['2'] = 1500  # Roll
        vehicle.channels.overrides['3'] = 1300  # Pitch # move para tras 
        vehicle.channels.overrides['4'] = 1500  # Yaw
    elif color == "red":
        # Mover o drone para esquerda
        vehicle.channels.overrides['1'] = 1500  # Throttle
        vehicle.channels.overrides['2'] = 1300  # Roll # move para esquerda
        vehicle.channels.overrides['3'] = 1500  # Pitch
        vehicle.channels.overrides['4'] = 1500  # Yaw
    elif color == "green_after_red":
        # Pousar o drone
        vehicle.mode = VehicleMode("LAND")
    else:
        print("desconhecida coloração: {}".format(color))

# precisamos do principal que é uma função que trata a cor recebida 
# agora vem o loop de movimentação que captura continuamente os frames e filtra a cor de interesse e a partir disso realiza a movimentação
# a função vai ser chamada toda vez que o parametro msg for chamado/recebido. Esse parametro é a imagem capturada pelo sensor atraves do nó ros criado anteriormente  
def image_callback(msg):
    # primeiro ao receber a imagem do ros vamos tentar converte-la para o formato do open cv, se não conseguirmos entramos um except e paramos a função
    try:
        # Converter a imagem do formato ROS para OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    # Na imagem capturada pela camera há varios pixels com diferentes valores RGB, porém é dificil de trabalhar com a cor nesse formato por isso convertemos para HSV 
    # Converter a imagem para o espaço de cor HSV que é um espaço de cores que é uma representação de cor em três componentes: (Hue,Saturation,Value) = saturação, matiz e valor. Essa é uma representação muito utilizada na visão computacional para manipular imagens RGB.
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Definir os limites inferior e superior para cada cor que tambem é um ferramente muito usada em tarefas de visão computacioanl. Ao definir esses limites de cada cor no espaço HSV podemos extrair mais facilmente os pixels de interesse que representam cada cor.
    green_lower = np.array([40, 50, 50])
    green_upper = np.array([70, 255, 255])
    blue_lower = np.array([100, 50, 50])
    blue_upper = np.array([130, 255, 255])
    pink_lower = np.array([140, 50, 50])
    pink_upper = np.array([170, 255, 255])
    red_lower = np.array([0, 50, 50])
    red_upper = np.array([10, 255, 255])
    red_lower_2 = np.array([170, 50, 50])
    red_upper_2 = np.array([180, 255, 255])
    # são limites já estabelecidos para essas cores no HSV 

    # Aplicar a máscara para detectar as cores, em que mascara consiste numa imagem binaria que é usada para selecionar partes da imagem que estamos interessados em analisar. Ela é construida, nesse caso, utilizando os limites inferiores e superiores de cada cor definidos anteriormente.
    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
    blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
    pink_mask = cv2.inRange(hsv_image, pink_lower, pink_upper)
    red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
    red_mask_2 = cv2.inRange(hsv_image, red_lower_2, red_upper_2)
    red_mask = cv2.bitwise_or(red_mask, red_mask_2)

    # Encontrar os contornos de cada cor detectada, ou seja o pedaço da imagem que tem a cor de interesse
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pink_contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # a função retorna duas coisas mas so temos interesse em uma

    # Desenhar os contornos na imagem original. Isso pode ser util para visualizar se a detecção está saindo como esperado
    cv2.drawContours(cv_image, green_contours, -1, (0, 255, 0), 2)
    cv2.drawContours(cv_image, blue_contours, -1, (255, 0, 0), 2)
    cv2.drawContours(cv_image, pink_contours, -1, (255, 192, 203), 2)
    cv2.drawContours(cv_image, red_contours, -1, (0, 0, 255), 2)

    # Finalmente tratamos a imagem vamos para parte de movimentação
    # se encontrarmos contorno de determinada cor nas imagens vamos chamar a função de movimento e movimentar o drone. 
    if len(green_contours) > 0: 
        move_drone("green")
    elif len(blue_contours) > 0:
        move_drone("blue")
    elif len(pink_contours) > 0:
        move_drone("pink")
    elif len(red_contours) > 0:
        move_drone("red")
        # Esperar 2 segundos para que o drone se mova antes de detectar a cor verde novamente
        rospy.sleep(2)
        # agora verificar se a cor verde aparece é seguindo os passos anteriores de detecção de cores
        # Verificar se a cor verde é detectada após o vermelho
        green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(green_contours) > 0:
            move_drone("green_after_red")
    else:
    print("Nenhuma cor detectada")

def main():

    # configurações basicas para inicializar um drone   
    connection_string = 'udp:127.0.0.1:14550' #conexão com o veiculo drone atraves do protocolo de comunicação UDP 
    vehicle = connect(connection_string, wait_ready=True) #criando a conexão entre o computador e o veículo drone através do Mavlink

    # Inicializa a ponte do OpenCV para o ROS
    bridge = CvBridge()

    # Inicialização do ROS com o nó chamado color detection 
    rospy.init_node('color_detection', anonymous=True)

    # esperar o veiculo esta pronto para "armar" o drone 
    while not vehicle.is_armable:
        time.sleep(1)

    # para o drone se mover só na simulação ele precisa estar no modo guided e armed
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # ter certeza que ele está "armed" antes de decolar 
    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(2) # subindo o drone

    # cria um objeto a partir do open cv que captura imagens de uma fonte de vídeo que nesse caso é a câmera com isso podemos utilizar funções que leem essas imagens
    cap = cv2.VideoCapture(0) # não será mais util no codigo mas vou deixar

    # Subscrever ao tópico de imagem do drone. Estamos recebendo as mensagens de imagem do drone e quando uam mensagem é recebida a função será chamada para essa imagem.

    rospy.Subscriber('/bebop/image_raw', Image, image_callback)

    # Loop ROS
    rospy.spin()

if __name__ == '__main__':
    main()




    



