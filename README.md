<h1 align="center">Sistema de navegación para una silla de ruedas inteligente </h1>

# Descripción
El sistema de navegación implementado fue para una silla de ruedas, sin embargo se utilizó un create y un kinect para las pruebas e implementación,
usando principalmente como sistema operativo Ubuntu 17.04 y ROS-Melodic. Se utilizó el algoritmo RRT y informedRRT* para la planificación de rutas,
más aparte se utilizó quadtrees para la optimización de consultas sobre puntos y para la parte de comprobación de obstáculos se usó un Analizador 
Digital Diferencial.
![imagen](https://github.com/hermes76/Navigation-system-for-an-intelligent-electric-wheelchair/assets/44623908/98e67d86-5be4-4bf3-aa75-be00ceffeb59)

# Requisitos
- [Ubuntu 17.04]()
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Create robot](https://github.com/AutonomyLab/create_robot)

# Instalación
Para clonar este repositorio utiliza los siguientes comandos
```
$ cd catkin_ws
$ cd src
$ git clone <este repositorio>
$ cd ..
$ catkin_make
```

# Módulo de mapeo
Para mapear una habitación o algun lugar usando el kinect es necesario ejecutar los siguientes paquetes,abre una nueva terminal y escribre lo siguiente:
```
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch ewc_slam ewc_slam.launch slam_methods:=gmapping
```
Para mover el create con el teclado ejecuta los siguientes comandos en una nueva terminal:
```
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun ewc_teleop ewc_teleop_key
```

# Módulo de navegación
Para ejecutar el módulo de navegación se deben de ejecutar los siguientes comandos
```
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch ewc_navigation configuration.launch
```
Una vez que cargue el mapa en la herramienta de Rviz deberás de colocar los puntos delimitantes para la navegación con el botón 
![imagen](https://github.com/hermes76/Navigation-system-for-an-intelligent-electric-wheelchair/assets/44623908/2748dd6b-3bb2-417c-84e3-ad89df848c00)

Para colocar los 2 puntos siempre debes de empezar desde la esquina inferior derecha y poner el punto de la esquina superior izquierda.

![imagen](https://github.com/hermes76/Navigation-system-for-an-intelligent-electric-wheelchair/assets/44623908/0b9fb71c-0cc6-47a2-9a64-a66459b708ed)

listo después solo introduce la posición estimada y la posición a donde deseas llega :shipit:
![imagen](https://github.com/hermes76/Navigation-system-for-an-intelligent-electric-wheelchair/assets/44623908/684a8693-3fc8-48ab-b26d-891f8bc9000c)




