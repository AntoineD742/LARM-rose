# Challenge 1

Bienvenue sur notre branche concernant le challenge 1 de l'UV LARM !

## Installation

Récupérez notre code :
``` bash
git clone https://github.com/AntoineD7420/LARM-rose
```

Passez sur la branche challenge1 :
``` bash
git checkout challenge1
```

## Dépendance

**Attention** pour que tout fonctionne normalement il est nécessaire d'installer sur votre machine le package **`mb6-bot`**. Pour l'installer veuillez suivre les consignes disponibles avec ce lien :
https://bitbucket.org/imt-mobisyst/mb6-tbot/src/master/

## Lancement de la simulation

Pour lancer la simulation, éxecutez les commandes suivantes depuis le catkin-workspace :
``` bash
catkin_make
source devel/setup.bash
roslaunch grp-rose challenge1_simulation.launch
```

## Mise en route du robot

Pour mettre en route le robot, éxecutez les commandes suivantes depuis le catkin-workspace :
``` bash
catkin_make
source devel/setup.bash
roslaunch grp-rose challenge1_turtlebot.launch
```