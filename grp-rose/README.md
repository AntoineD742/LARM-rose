# Challenge 2

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

## Dépendance

**Attention** pour que tout fonctionne normalement il est nécessaire d'installer sur votre machine le package **`mb6-bot`**. Pour l'installer veuillez suivre les consignes disponibles avec ce lien :
https://bitbucket.org/imt-mobisyst/mb6-tbot/src/master/

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