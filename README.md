# Installations et Stack Technique Guide

## Introduction

Ce fichier est un guide à suivre pour installer le stack technique requis pour ce projet. Voici le stack technique :
- Linux Ubuntu 22.04

- Ardupilot (ArduCopter==4.5.7, Rover==4.5.7, Tracker==4.5.7) : https://ardupilot.org/dev/docs/building-setup-linux.html

- Gazebo Harmonic : https://gazebosim.org/docs/harmonic/install/

- ROS 2 Humble : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

- Ardupilot Gazebo : https://ardupilot.org/dev/docs/sitl-with-gazebo.html

- Pip requirements : dans Requirements.txt
- matplotlib==3.9.3

## Sommaire

- [Installations et Stack Technique Guide](#installations-et-stack-technique-guide)
  - [Introduction](#introduction)
  - [Sommaire](#sommaire)
  - [1. Simulation mono-drone](#1-simulation-mono-drone)
    - [A. Installer ArduPilot SITL](#a-installer-ardupilot-sitl)
    - [B. Installer Gazebo Harmonic](#b-installer-gazebo-harmonic)
    - [C. Installer ArduPilot Gazebo](#c-installer-ardupilot-gazebo)
  - [2. Simulation Multi-Drones](#2-simulation-multi-drones)
    - [A. Installer ROS2](#a-installer-ros2)
    - [B. Installer Micro ROS](#b-installer-micro-ros)
    - [C. Utiliser ROS2 avec ArduPilot SITL](#c-utiliser-ros2-avec-ardupilot-sitl)
    - [D. Utiliser ROS2 avec SITL](#d-utiliser-ros2-avec-sitl)
    - [E. Utiliser Gazebo + SITL + ROS2](#e-utiliser-gazebo--sitl--ros2)
    - [F. Installer MAVROS pour ROS2](#f-installer-mavros-pour-ros2)
    - [G. Utiliser ROS, MAVROS pour controler le drone](#g-utiliser-ros-mavros-pour-controler-le-drone)
    - [F. Utiliser Cartographer SLAM en ROS2, SITL et Gazebo](#f-utiliser-cartographer-slam-en-ros2-sitl-et-gazebo)
    - [G. Simulation de plusieurs drones sans ROS](#g-simulation-de-plusieurs-drones-sans-ros)
    - [H. Simulation de plusieurs drones AVEC ROS](#h-simulation-de-plusieurs-drones-avec-ros)




## 1. Simulation mono-drone

### A. Installer ArduPilot SITL
Source : https://ardupilot.org/dev/docs/building-setup-linux.html
Installation ArduPilot SITL :

```shell
cd ~
sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Il faut drestart son PC a ce moment la ;)

Vérification ArduPilot:
(shell Linux)
```shell
sim_vehicle.py -v ArduCopter -f quad --console --map
```

(shell MAV)
```
mode guided
arm throttle
takeoff 10
```

### B. Installer Gazebo Harmonic
Source : https://gazebosim.org/docs/harmonic/install/
Installation Gazebo Harmonic (Ubuntu) :
```shell
cd ~
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

Installation Gazebo Harmonic (MacOS) :
```
brew tap osrf/simulation
brew install gz-harmonic
```


Vérification installation Gazebo H :
```shell
gz sim -v4 -r shapes.sdf
# (vérifier que ça lance bien une simu Gazebo avec des formes)
```


### C. Installer ArduPilot Gazebo

Source : https://ardupilot.org/dev/docs/sitl-with-gazebo.html
Installation Ardupilot Gazebo (Linux):
```shell
cd ~
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
mkdir -p gz_ws/src && cd gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
export GZ_VERSION=harmonic
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```


Installation Ardupilot Gazebo (MacOS):
```
brew update
brew install rapidjson
brew install opencv gstreamer
mkdir -p gz_ws/src && cd gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
export GZ_VERSION=harmonic
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

Vérification de l'installation du plugin :
```shell
Dans le premier terminal :
gz sim -v4 -r iris_runway.sdf
```

Dans le deuxième terminal :
(shell Linux)
```shell
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

(shell MAV)
```
mode guided
arm throttle
takeoff 10
(Le drone doit décoller dans Gazebo)
```


## 2. Simulation Multi-Drones


### A. Installer ROS2
ROS ROADMAP : 
Source : https://docs.ros.org/en/humble/Tutorials.html
- Installer ROS2
- Tutoriels de base


Source : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
Installation ROS2 Humble (Linux) :
```shell
cd ~
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash
```

Vérification de l'installation de Ros2 Humble :
(Terminal 1) :
``` shell
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

(Terminal 2) :
``` shell
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Source : https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
Vérification versions Ros 2 Humble :

```shell
printenv | grep -i ROS

# Vérifier que :
# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
# ROS_DISTRO=humble
```


POUR COMPRENDRE ROS2 : SUIVRE LES TUTORIELS EXPLIQUES ICI : https://docs.ros.org/en/humble/Tutorials.html
DES DETAILS SONT EXPLIQUES DANS LE FICHIER ROS2_ONBOARDING.txt


Installer Colcon (Linux) :
```shell
sudo apt install python3-colcon-common-extensions
pip install setuptools==58.2.0
```

Vérification de l'installation de colcon sur Linux :
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/ros2/examples src/examples -b humble
colcon build --symlink-install --parallel-workers 2
colcon test
cd ~
rm -rf ros2_ws/
# Si un popup apparait avec "colcon build succesful", c'est une réussite !
```

### B. Installer Micro ROS
Installer Micro ROS : 
Source : 
https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS
https://micro.ros.org/docs/tutorials/core/first_application_linux/


Remarque : il est possible qu'il soit nécessaire de run la commande suivante avant d'installer la suite : ```sudo rosdep init``` et ```rosdep update```

```shell
cd ~
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
# Install pip
sudo apt-get install python3-pip
# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```


### C. Utiliser ROS2 avec ArduPilot SITL
Source : https://ardupilot.org/dev/docs/ros.html

```shell
cd ~
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
```

Tester la bonne installation jusqu'ici:
```shell
source ~/.bashrc
microxrceddsgen -help
```

Supposé renvoyer : 
```shell
# microxrceddsgen usage:
#     microxrceddsgen [options] <file> [<file> ...]
#     where the options are:
#             -help: shows this help
#             -version: shows the current version of eProsima Micro XRCE-DDS Gen.
#             -example: Generates an example.
#             -replace: replaces existing generated files.
#             -ppDisable: disables the preprocessor.
#             -ppPath: specifies the preprocessor path.
#             -I <path>: add directory to preprocessor include paths.
#             -d <path>: sets an output directory for generated files.
#             -t <temp dir>: sets a specific directory as a temporary directory.
#             -cs: IDL grammar apply case sensitive matching.
#     and the supported input files are:
#     * IDL files.
```


Suite de l'installation :
```shell
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests
colcon build --packages-select fastcdr
pip install numpy==1.24.0
```

Attention si ça fail, suivre le tutoriel décrit dans depannage_ardupilot_ROS.txt !



Test de la bonne construction du workspace :
```shell
cd ~/ardu_ws
source ./install/setup.bash
colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
colcon test-result --all --verbose
```

### D. Utiliser ROS2 avec SITL
Source : https://ardupilot.org/dev/docs/ros2-sitl.html

```shell
cd ~/ardu_ws/
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```


### E. Utiliser Gazebo + SITL + ROS2
Source : https://ardupilot.org/dev/docs/ros2-gazebo.html
```shell
cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
export GZ_VERSION=harmonic
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
```


Tester que ça a fonctionné :
``` shell
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

``` shell
mavproxy.py --console --map --aircraft test --master=:14550
```


TESTS pour voir si on peut controler le drone via ROS2

```shell
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: True}"
ros2 service call /ap/prearm_check std_srvs/srv/Trigger
ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 10.5}"
```

Le drone décolle-t-il bien ?

https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS


Attention, il n'est plus nécessaire d'utiliser MAVROS pour piloter le drone, on peut utiliser la commande suivante :
La commande suivante permet de piloter la velocité du drone
```shell
ros2 topic pub /ap/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, twist: {linear: {x: 5.0, y: 0.0, z: 5.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Rappel, pour savoir comment utiliser un service/action :
1. Premièrement, savoir quels services existent : ``` ros2 service list ```
2. Deuxièmement, connaitre le type d'un service en particulier : 
  ```shell
  ros2 service type <nom_du_service>
  # Renvoie :
  # <type_du_service_en_question>
  ```
3. Enfin, 
   ```
   ros2 interface show <type_du_service_en_question>
   # Renvoie :
   # description du service
   #
   # <type de l'input 1> <nom_input_1>
   # <type de l'input 2> <nom_input_2>
   # ...
   # <type de l'input m> <nom_input_m>
   # ---
   # <type de l'output 1> <nom_output_1>
   # <type de l'output 2> <nom_output_2>
   # ...
   # <type de l'output p> <nom_output_p>


   # ------------------------------
   # Exemple
   # ------------------------------

   # ros2 interface show mavros_msgs/srv/CommandBool

   # bool value
   # ---
   # bool success
   # uint8 result
   ```
4. Utiliser la commande comme tel : ``` ros2 service call <nom_du_service> <type_du_service_en_question> "{<nom_input_1> : <valeur_à_affecter>, <nom_input_2> : <valeur_à_affecter>}"```
   Tous les inputs ne doivent pas forcément être spécifiés, des valeurs par défaut sont mises à leur place si un couple "input"/"valeur" n'est pas mis dans le dictionnaire. 




### F. Installer MAVROS pour ROS2



Source : https://github.com/mavlink/mavros/blob/ros2/mavros/README.md
```shell
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon
# 1. Go to your workspace
cd ~/ardu_ws
# 2. Install MAVLink
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
# 3. Install MAVROS: get source (upstream - released)
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos
# 4. Create workspace & deps
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos
rosdep install --from-paths src --ignore-src -y
# 5. Install GeographicLib datasets:
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
# Build source
colcon build --packages-up-to ardupilot_gz_bringup
colcon build --packages-select mavlink
colcon build --packages-select mavros
```

Tester que ça a fonctionné :
```shell
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```


### G. Utiliser ROS, MAVROS pour controler le drone


``` shell
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

```shell
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

```shell
# Mode Guided
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
# Arm Motors
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
# Takeoff
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}" 
```

Dans ce même dernier terminal, une fois le décollage terminé, on peut commander le drone en position :
```shell
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 0.0, z: 5.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```




### F. Utiliser Cartographer SLAM en ROS2, SITL et Gazebo
```shell
cd ~/ardu_ws/src
git clone git@github.com:ArduPilot/ardupilot_ros.git
cd ~/ardu_ws
rosdep install --from-paths src --ignore-src -r --skip-keys gazebo-ros-pkgs
cd ~/ardu_ws
source ./install/setup.bash
colcon build --packages-up-to ardupilot_ros ardupilot_gz_bringup
```

Utiliser :
(Terminal 1)
``` shell
source ~/ardu_ws/install/setup.sh
ros2 launch ardupilot_gz_bringup iris_maze.launch.py
```

(Terminal 2)
```shell
source ~/ardu_ws/install/setup.sh
ros2 launch ardupilot_ros cartographer.launch.py
```

(Terminal 3)
```shell
mavproxy.py --console --map --aircraft test --master=:14550
```


### G. Simulation de plusieurs drones sans ROS

- Copier le dossier Iris_with_gimball des modèles gazebo (ardupilot_gazebo/Models) pour faire différents dossiers drone1, drone2, …

- Dans chacun des dossiers modifier :
	- model.config : changer le nom du modèle dans la balise name
	
	- model.sdf : changer le nom du modèle dans la balise model en haut du fichier
			changer la valeur de fdm_port_in pour 9012, 9022, …  en dessous de la balise  plugin ArduPilotPlugin (ligne 191)

- Dans le dossier ardupilot_gazebo/Worlds:
	copier le fichier iris_runway pour créer un nouvel environnement contenant les drones en copiant la balise include de iris_with_gimball 
	
- Dans le dossier ardupilot/Tools/autotest/default_params :
	Créer une copie du fichier gazebo-iris.parm pour chaque drone créé précédemment et ajouter la ligne SYSID_THISMAV n, avec n le numéro du drone.

Dans le fichier ardupilot/Tools/autotest/pysim/vehicleinfo.py : créer une copie de la partie correspondant à gazebo-iris pour chaque drone créé précédemment et changer gazebo-iris pour le nom du drone.

Les drones sont alors configurés, il faut maintenant lancer la simulation :

Dans un premier terminal :
gz sim -v4 -r «Nom du fichier world».sdf

Pour chacun des drones :
sim_vehicle.py -v ArduCopter -f «Nom du drone» --model JSON --console -In (remplacer n par l’identifiant du drone)


```shell
gz sim -v4 --render-engine ogre iris_runway_swarm.sdf
# Si erreur dans l'étape suivante, run cette commande ci :
gz sim -v4 -r iris_runway_swarm.sdf
```

```shell
sim_vehicle.py -v ArduCopter -f swarm-drone1 --model JSON --console -I1
```

```shell
sim_vehicle.py -v ArduCopter -f swarm-drone1 --model JSON --console -I1 --out=tcpin:0.0.0.0:8100
```


### H. Simulation de plusieurs drones AVEC ROS

Aller dans ~/ardu_ws/src/ardupilot_gz/ardupilot_gz_bringup/launch/
Copier le fichier iris_runway.launch.py
Coller le dans le même répertoire
Renommer ce fichier en "iris_runway_swarm.launch.py"
Ouvrir le fichier iris_runway_swarm.launch.py
A la ligne 78, remplacez le code suivant :
``` python
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_runway.sdf"}'
```
En :
``` python
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_runway_swarm.sdf"}'
```

Aller dans ~/ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/
Copier le fichier "iris_runway.sdf"
Coller le dans le même répertoire
Renommer ce fichier en "iris_runway_swarm.sdf"


Aller dans ~/ardu_ws/src/ardupilot_gz/ardupilot_gz_bringup/launch/robots
Copier le fichier "iris.launch.sdf"
Coller le dans le même répertoire
Renommer ce fichier en "drone1ros.launch.sdf"
Ouvrir ce fichier

Changer à la ligne 89 remplacez le code suivant :
``` python
                "gazebo-iris-gimbal.parm",
```
En :
``` python
                "drone1ros.parm",
```


Changer à la ligne 118 remplacez le code suivant :
``` python
        pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
```
En :
``` python
        pkg_ardupilot_gazebo, "models", "drone1ros", "model.sdf"
```

Aller dans ~/ardu_ws/src/ardupilot_gazebo/config
Copier le fichier "gazebo-iris-gimbal.parm"
Coller le dans le même répertoire
Renommer ce fichier en "drone1ros.parm"
Ouvrir ce fichier
Ajouter la ligne suivante :
```
SYSID_THISMAV 1
```




install/ardupilot_sitl/share/ardupilot_sitl/config/default_params




``` shell
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway_swarm.launch.py
```

Suivre les mêmes étapes qu'à la phase précédente :
Modifier les fichiers world dans le répertoire suivant :
/home/gauth/ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo





ROS2 Commands :

Services :


Changer de mode :

Mode 1 : ACRO
``` shell
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 1}"
```

Mode 2 : ALT_HOLD
``` shell
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 2}"
```

Mode 3 : AUTO
``` shell
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 3}"
```

Mode 4 : GUIDED
``` shell
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
```


Source : https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/multi_mavros_drones.md

ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2ros tgt_system:=2


Avant :
(Terminal 1)
``` shell
source ~/ardu_ws/install/setup.sh
ros2 launch ardupilot_gz_bringup iris_maze.launch.py
```

Explication du code : ros2 launch, lance un programme ros2
ardupilot_gz_bringup est le nom du répertoire
iris_maze.launch.py est le nom du fichier à executer dans le répertoire launch de ardupilot_gz_bringup


Maintenant :
(Terminal 1)
``` shell
source ~/ardu_ws/install/setup.sh
ros2 launch ardupilot_gz_bringup iris_runway_swarm.launch.py
```

```
colcon build --packages-up-to ardupilot_gz_bringup
colcon build --packages-select mavlink
colcon build --packages-select mavros
```

```
ros2 launch mavros multi-apm.launch 
```


``` shell
# Mode Guided
ros2 service call /drone1/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /drone2/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
ros2 service call /drone3/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"


# Arm Motors
ros2 service call /drone1/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /drone2/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /drone3/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"


# Takeoff
ros2 service call /drone1/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}" 
ros2 service call /drone2/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}" 
ros2 service call /drone3/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 5.0}" 


ros2 topic pub /drone1/setpoint_position/local geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 0.0, z: 5.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```
