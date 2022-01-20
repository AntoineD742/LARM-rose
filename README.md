# Challenge 2 - Groupe Rose

Bienvenue sur notre branche concernant le challenge 2 de l'UV LARM !

## Installation

Récupérez notre code :
``` bash
git clone https://github.com/AntoineD7420/LARM-rose
```

Passez sur la branche challenge1 :
``` bash
git checkout challenge2
```

## Dépendances

**Attention** pour que tout fonctionne normalement il est nécessaire d'installer sur votre machine le package **`mb6-bot`**. Pour l'installer veuillez suivre les consignes disponibles avec ce lien :
https://bitbucket.org/imt-mobisyst/mb6-tbot/src/master/

Il faut aussi installer les **`drivers`** et les **`packages`** ros liés à la caméra **`RealSense`**.
Pour installer les drivers veuillez suivre les consignes disponibles avec ce lien :
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

Ensuite veuillez installer les librairies et outils suivants :
``` bash
sudo apt install \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
```
Enfin, veuillez installer les packages ros suivants :
``` bash
sudo apt-get install \
    ros-noetic-librealsense2 \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description
```

## But de ce challenge

Le but du challenge 2 est que le robot génére une map sur Rviz via le topic `/map` en récupérant les données d'un robag. En plus de générer cette map le robot doit aussi détecter des bouteilles oranges et les placer sur la map grâce à des marqueurs verts. On peut voir la position de ces marqueurs en écoutant le topic `/bottle`.
Pour détecter les bouteilles nous avons utilisé une caméra RealSense Intel et une méthode de détection par seuillage colorimétrique avec conversion HSV.

## Lancement de l'exploration

Pour lancer l'exploration, éxecutez les commandes suivantes depuis le catkin-workspace :
``` bash
catkin_make
source devel/setup.bash
roslaunch grp-rose challenge2.launch
```

Il faut ensuite lancer le rosbag avec la commande suivante :
``` bash
rosbag play --clock chemin/vers/le/rosbag
```

Il est aussi possible d'écouter le topic /bottle pour voir la position de ces dernières :
``` bash
rostopic echo /bottle
```
