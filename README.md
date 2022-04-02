# Robótica Taller II

Repositorio que incluye los recursos utilizados por el grupo 11 que da solución al taller 2 "Diseño de un robot diferencial" del curso de robótica.

- [Robótica Taller II](#robótica-taller-ii)
- [Instalación](#instalación)
  * [Requisitos](#requisitos)
  * [Dependencias](#dependencias)
    + [Librerías](#librerías)
    + [Paquetes de ROS](#paquetes-de-ros)
    + [Librerías Arduino](#librerías-arduino)
  * [Instalación](#instalación-1)
- [Ejecución](#ejecución)
  * [Ejecución de rosserial](#ejecución-de-rosserial)
  * [Ejecución de differential-drive](#ejecución-de-differential-drive)
  * [Ejecución de los nodos desarrollados](#ejecución-de-los-nodos-desarrollados)
    + [Punto 1](#punto-1)
    + [Punto 2](#punto-2)
    + [Punto 3](#punto-3)
    + [Punto 4](#punto-4)
- [Troubleshooting](#troubleshooting)
  * [ModuleNotFoundError](#modulenotfounderror)
    + [Numpy](#numpy)
    + [Pillow](#pillow)
    + [Pynput](#pynput)
    + [Rospy](#rospy)
    + [Tkinter](#tkinter)
    + [rospkg](#rospkg)

# Instalación
## Requisitos
- Linux Ubuntu 18.04
- ROS Melodic
- Python >= 3.6

## Dependencias
### Librerías

- [Numpy][Numpy]
- [Pillow][Pillow]
- [Pynput][Pynput]
- [Rospy][Rospy]
- [Tkinter][Tkinter]

### Paquetes de ROS
- [rospkg][rospkg]
- [geometry_msgs][geometry_msgs]
- [std_msgs][std_msgs]
- [rosserial][rosserial]
- [differential-drive][differential-drive]

### Librerías Arduino
- [rosserial][rosserial]
- [motor-drive][motor-drive]
- [PID][PID]

## Instalación

1. Subir código "MotorConPID.ino" al arduino UNO.

2. Crear un espacio de trabajo de ROS en la Raspberry Pi donde se encuentre instalado el paquete differential-drive.

3.  Crear un espacio de trabajo de ROS en el computador local y descargar el paquete mi_robot_11 dentro de este.

# Ejecución

## Ejecución de rosserial

1. Lanzar un roscore

```bash
roscore
```

2. Ejecutar rosserial

```bash
rosrun rosserial_python serial_node.py 
```

## Ejecución de differential-drive

1. Dentro del entorno de trabajo de ROS cargar las variables de entorno

```bash
source devel/setup.bash
```

2. Ejecutar paquete de nodos

```bash
roslaunch pi_ws robot_pkg drive.launch
```

## Ejecución de los nodos desarrollados

### Punto 1

1. Dentro del entorno de trabajo de ROS cargar las variables de entorno

```bash
source devel/setup.bash
```

2. Ejecutar nodo

```bash
rosrun mi_bot_11 punto1.py
```

3. Ingresar por consola la velocidad lineal y angular deseada 

4. Utilizar el teclado para mover al robot (w, s, q, e)

### Punto 2
1. Dentro del entorno de trabajo de ROS cargar las variables de entorno

```bash
source devel/setup.bash
```

2. Ejecutar nodo

```bash
rosrun mi_bot_11 punto2.py
```

3. En la ventana emergente índicar si se quiere guardar el recorrido que se realizará (en caso de que se quiera guardar el recorrido se debe ejecutar el punto 3 para mover el robot en vez del punto 1)

4. Si se selecciona la opción de guardar el recorrido, entonces, debe seleccionar el nombre y la ubicación deseada

5. Al mover el robot, se comenzará a graficar la posición de esté en tiempo real. Se puede guardar en cualquier momento la gráfica realizada índicando el nombre y la ubicación deseada.

### Punto 3
1. Dentro del entorno de trabajo de ROS cargar las variables de entorno

```bash
source devel/setup.bash
```

2. Para que el punto 3 funcione, la interfaz gráfica desarrollada en el punto 2 debe estar corriendo y se debe haber seleccionado que se quiere guardar el recorrido, así:

```bash
rosrun mi_bot_11 punto2.py
```

3. Correr el nodo correspondiente al punto 3.

```bash
rosrun mi_bot_11 punto3.py
```

4. Utilizar el teclado para mover al robot (w, s, q, e)

5. En el momento que se quiera detener el robot, utilizar control+z. La información relacionada al recorrido se guardó en la ubicación especificada en el punto 2.

### Punto 4

1. Dentro del entorno de trabajo de ROS cargar las variables de entorno

```bash
source devel/setup.bash
```

2. Para que el punto 4 funcione, la interfaz gráfica desarrollada en el punto 2 debe estar corriendo, así:

```bash
rosrun mi_bot_11 punto2.py
```

3. Correr el nodo correspondiente al punto 4:

```bash
rosrun mi_bot_11 punto4.py
```

4. En la interfaz gráfica seleccionar la opción 'cargar recorrido' y seleccionar el folder relacionado al recorrido que se quiere reproducir.

# Troubleshooting
En esta sección se encuentra la solución a los problemas que pueden ocurrir al momento de intentar utilizar este paquete.

## ModuleNotFoundError
Este error ocurre cuando una de las librerías o paquetes descritos en la sección de dependencias no está instalado.

Para las bibliotecas de python se recomienda instalarlas a través de pip3.

*Si no cuenta con esta herramienta, puede instalarla a través del siguiente comando:*

```bash
sudo apt install python3-pip
```
### Numpy

Instalando Numpy a través de pip:

```bash
pip3 install numpy
```

### Pillow

Instalando Pillow a través de pip:

```bash
pip3 install pillow 
```

### Pynput

Instalando pynput a través de pip:

```bash
pip3 install pynput 
```

### Rospy

Si este error ocurre revisar que se hayan cargado las variables de entorno del espacio de trabajo de ROS y si el error persiste revisar la instalación de ROS.

### Tkinter

Instalando Tkinter para Python3 a través de pip:

```bash
sudo apt-get install python3.x-tk
```

Donde x corresponde a la versión de python correspondiente. 

### rospkg
Instalando algunos prerrequsitos para utilizar python3 con ROS:

```bash
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

[Numpy]: https://numpy.org "Numpy"
[Pynput]: https://pypi.org/project/pynput/ "Pynput"
[Rospy]: http://wiki.ros.org/rospy "Rospy"
[Tkinter]: https://docs.python.org/3/library/tkinter.html "Tkinter"
[geometry_msgs]: http://wiki.ros.org/geometry_msgs "geometry_msgs"
[rospkg]: http://wiki.ros.org/rospkg "rospkg"
[std_msgs]: http://wiki.ros.org/std_msgs "std_msgs"
[Pillow]: https://python-pillow.org "Pillow"
[CoppeliaSim]: https://coppeliarobotics.com "CoppeliaSim"

[rosserial]: http://wiki.ros.org/rosserial "rosserial"
[differential-drive]: https://github.com/jfstepha/differential-drive "differential-drive"
[rosserial]: https://www.arduino.cc/reference/en/libraries/rosserial-arduino-library/ "rosserial"
[motor-drive]: https://www.arduino.cc/reference/en/libraries/motor-driver-library/ "motor-drive"
[PID]: https://playground.arduino.cc/Code/PIDLibrary/ "PID"
