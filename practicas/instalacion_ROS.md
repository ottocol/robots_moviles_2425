# Instrucciones de instalación de ROS

Para las prácticas, en principio os vale cualquier versión de ROS 1, no os recomiendo usar ROS 2 ya que os complicaría bastante probar el código con los turtlebots del laboratorio, y los ejemplos que pondremos en clase siempre serán con ROS 1.

Tenéis varias opciones para usar ROS 1 en vuestro ordenador:


- Usar Docker, como tenemos en los laboratorios
- Usar una máquina virtual en cualquier SO
- Instalar ROS en linux nativo
- Opciones intermedias como instalar ROS en Windows usando WSL (Windows Subsystem for Linux). Aquí no lo describiremos con detalle pero podéis encontrar tutoriales en Youtube buscando por ejemplo "instalar Gazebo y ROS con WSL" (como siempre encontraréis más información si buscáis en inglés).

## Opción a) Usar docker

Docker permite crear y gestionar contenedores, que son como máquinas virtuales ligeras. Funciona en prácticamente todas las plataformas. Si queréis usarlo en Windows necesitáis que esté instalado el WSL (Windows Subsystem for Linux) para dar soporte a contenedores linux.

Puedes buscar fácilmente en Internet cómo instalar Docker Desktop, que incluye las herramientas en línea de comandos y la interfaz gráfica.

Una vez instalado, arranca Docker Desktop y ve al apartado de "Images". En la barra de búsqueda de fondo azul de la parte superior de la ventana busca la imagen llamada `ottocol/ros-melodic-tb2`. Contiene una instalación de ROS Melodic con soporte para Turtlebot 3 y Turtlebot 2. Pulsa sobre el botón "Pull" para descargar la imagen al disco, ocupa unos 5Gb.

> Una **imagen** es como un *snapshot* de una máquina virtual. Cuando la arrancamos lo que generamos en marcha es un **contenedor**. Lo que hagamos en este contenedor no afecta a la imagen original de modo que si arrancamos un contenedor nuevo a partir de la imagen contendrá exactamente lo mismo que la imagen original. No obstante los contenedores se pueden parar y poner en marcha y conservan los cambios, así que si quieres instalar software

Una vez se haya descargado la imagen, para ponerla en marcha abre una terminal (en windows puedes hacerlo con la orden `cmd`) y en ella escribe:

```bash
docker run -it --name=ros_gazebo_desktop -m=4g -p 6080:80 -p 5900:5900  -e RESOLUTION=1080x720 -e USER=ubuntu -e PASSWORD=ubuntu ottocol/ros-melodic-tb2
```

Puedes cambiar o añadir los siguientes parámetros:

- `-m=4g` da 4Gb de memoria al contenedor, si tu máquina física tiene bastante memoria puedes intentar darle más
- `-e` como puedes ver especifica la resolución, puedes adaptarla a tus necesidades
- `-v directorio_windows:/home/ubuntu/data`: montar un directorio de windows en el contenedor(sustituye `directorio_windows` por cualquier directorio que tengas accesible desde la terminal) . Lo que almacenes en el directorio de windows será visible dentro del contenedor en `/home/ubuntu/data`. Así, puedes editar tu código en windows y ejecutarlo desde el linux del contenedor. Es una alternativa a modificar los datos dentro del contenedor, y es útil porque aunque destruyamos el contenedor los datos siguen estando al estar físicamente en el SO de "fuera".
- Si quieres que el contenedor se "autodestruya" cuando lo pares con `Ctrl-C` puedes añadir el argumento `--rm`. En ese caso no podrás rearrancarlo otra vez sino que tendrás que crear uno nuevo cada vez a partir de la imagen.

Conforme vaya arrancando el contenedor irá imprimiendo mensajes en la consola, una vez arrancado puedes verlo abriendo una ventana de un navegador y accediendo a `http://localhost:6080`. Debería mostrarse el escritorio del contenedor Docker, ejecutando una distribución de linux llamada Lubuntu. Puedes abrir una terminal y comprobar que todo va bien tecleando `roslaunch turtlebot_gazebo turtlebot_world.launch`, tras unos segundos debería aparecer el simulador Gazebo con una simulación de un Turtlebot 2 como los del laboratorio.

> Lo que estás viendo es el escritorio "remoto" del contenedor, que también se expone con un protocolo llamado VNC. Si lo prefieres puedes instalarte en tu ordenador un cliente VNC y conectar con `localhost:5079`, dependiendo de tu configuración es posible que obtengas alguna mejora de rendimiento.

Puedes parar el contenedor haciendo Crtl-C en la terminal en que hayas ejecutado `docker run` o bien desde la interfaz gráfica de Docker. Salvo que hayas añadido el argumento `--rm` puedes rearrancar el contenedor con `docker start ros_gazebo_desktop`.

## Opción b) Usar una máquina virtual

Os dejamos una [imagen de una máquina virtual en Virtualbox](https://drive.google.com/file/d/16537iqPFWIUc-hvZk7Ccb-JJEiCIRJhp/view?usp=share_link), con ROS Melodic en formato `.ova` (para abrir el enlace anterior **debes identificarte con tu cuenta de gcloud de la UA**). 

Para comprobar que todo funciona OK, una vez arrancada la máquina virtual abre una terminal (la tienes en el menú de inicio, en la opción "Herramientas del Sistema > LXTerminal") y ejecuta la siguiente orden:

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Debería aparecer una ventana del simulador Gazebo con un mundo visto desde arriba con una pared hexagonal, unos obstáculos cilíndricos blancos y un robot Turtlebot 3 tipo "waffle".

## Opción b) Instalar ROS Noetic en linux nativo


> La ventaja que tiene esta forma de instalación es el rendimiento, sobre todo para Gazebo, que en algunos ordenadores puede tener muy bajo rendimiento en una máquina virtual.

Las versiones de ROS 1 están soportadas oficialmente en Ubuntu, y cada versión de ROS es para una versión concreta de Ubuntu. Se recomienda usar la última versión de ROS 1, Noetic, ya que es la que requiere de un Ubuntu más actual. Para ello necesitas tener **Ubuntu 20.04**. Las anteriores versiones de ROS requieren versiones aún más antiguas de Ubuntu, que es poco probable que a estas alturas quieras tener en tu ordenador de forma nativa.

Tendrás que seguir estos pasos:

### 1. Instalar ROS

Sigue todas las [instrucciones de instalación oficiales](http://wiki.ros.org/noetic/Installation/Ubuntu) para Noetic (en el paso 1.4 instala la versión `desktop-full`, que tiene todas las herramientas de ROS incluidas las gráficas). Tras terminar estos pasos tendrás ROS instalado pero te faltará algún simulador de robots móviles.

### 2. Instalar el simulador de Turtlebot 3

No tenemos estos robots en el laboratorio pero nos serán útiles para probar algunos algoritmos en simulación.

Necesitarás ejecutar las siguentes instrucciones en una terminal:

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-simulations
```

Para comprobar que todo funciona, abre una nueva terminal y escribe:

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```

Debería aparecer una ventana del simulador Gazebo con un mundo con paredes en forma de hexágono y unos obstáculos cilíndricos blancos, visto desde arriba.

### 3. Instalar el simulador de Turtlebot 2

Estos robots sí los tenemos en el laboratorio, de modo que lo que pruebes en el simulador luego podrás probarlo con los robots reales. Por desgracia, no tienen soporte oficial para Noetic, lo que nos va a obligar a compilar los fuentes.

Para descargar los fuentes de los diversos repositorios necesarios de manera automática necesitas bajarte primero este fichero [tb2.rosinstall](tb2.rosinstall). Bájatelo y déjalo en tu directorio $HOME (tu directorio personal, o sea `/home/tu_nombre_de_usuario`). 

En una terminal, ejecuta las siguientes instrucciones:

```bash
#dependencias de paquetes de Ubuntu
sudo apt install libusb-dev libftdi-dev python-is-python3 pyqt5-dev-tools

#dependencias de paquetes de Noetic
sudo apt install ros-noetic-openslam-gmapping ros-noetic-joy ros-noetic-base-local-planner ros-noetic-move-base

#directorio para los fuentes
mkdir $HOME/tb2_ws
cd $HOME/tb2_ws
#se baja los fuentes de los repos especificados en el .rosinstall
wstool init src ../tb2.rosinstall

#Ñapa para arreglar un problema de dependencias
rm -rf src/ar_track_alvar/ar_track_alvar

#Compilar.Tardará! :)
catkin_make_isolated
```

Tardará un rato en compilar todos los fuentes. Una vez terminada la compilación, para terminar la configuración hay que hacer que tengamos accesibles los paquetes de Turtlebot2 cada vez que abrimos una terminal:

```bash
echo "source $HOME/tb2_ws/devel_isolated/setup.bash" >> ~/.bashrc
```

Cierra la terminal y abre una nueva para que los cambios tengan efecto (también puedes hacer `source ~/.bashrc`).

Para comprobar que funciona, ejecuta en una terminal:

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

Debería aparecer una ventana del simulador Gazebo con un robot Turtlebot2 rodeado de varios objetos. (La primera vez es normal que tarde ya que tiene que bajarse los modelos 3D de los objetos).






