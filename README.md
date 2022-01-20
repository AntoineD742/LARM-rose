# Challenge 3 - Groupe Rose

Bienvenue sur notre branche concernant le challenge 3 de l'UV LARM !

## Installation

Récupérez notre code :
``` bash
git clone https://github.com/AntoineD7420/LARM-rose
```

Passez sur la branche challenge3 :
``` bash
git checkout challenge3
```

## Dépendances

**Attention** pour que tout fonctionne normalement il est nécessaire d'installer sur votre machine le package **`mb6-bot`**. Pour l'installer veuillez suivre les consignes disponibles avec ce lien :
https://bitbucket.org/imt-mobisyst/mb6-tbot/src/master/

Il faut aussi installer les **`drivers`** et les **`packages`** ros liés à la caméra **`RealSense`**.
Pour installer les drivers veuillez suivre les consignes disponibles avec ce lien :
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

Ensuite, veuillez installer les librairies et outils suivants :
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

Le challenge 3 correspond en fait à la fusion du challenge 1 et 2. Le robot est censé se déplacer par lui-même en évitant les obstacles dans un environnement inconnu. En plus, il doit pouvoir générer une map dans le topic `/map` et détecter les bouteilles oranges présentes sur la map, il les affiche via des marqueurs verts sur Rviz via le topic `/bottle`. Nous avons rajouté la possibilité d'envoyer des goals au robot via Rviz et le topic `/move_base_simple/goal`. Cette fonctionnalité nous permet d'envoyer le robot à des endroits qu'il n'a pa pu explorer ou pour le débloquer en cas de soucis.

## Lancement de l'exploration en simulation

Pour lancer l'exploration, éxecutez les commandes suivantes depuis le catkin-workspace :
``` bash
catkin_make
source devel/setup.bash
roslaunch grp-rose challenge3_simulation.launch
```

Avec ce processus, le robot sera lancé en mode autonome, c'est-à-dire qu'il explorera la map tous seul grâce à un mouvement ricochet. Cependant, si vous voyez qu'il se retrouve bloqué ou qu'il n'a pas exploré une partie de la map, vous pouvez lui envoyer un goal via Rviz. Dès qu'un goal sera envoyé, le robot ne sera plus autonome, il atteindra ce dernier en évitant les obstacles et il s'arrêtera, il ne redémarrera que si un nouveau goal lui est envoyé.

## Lancement de l'exploration hors simulation

Pour lancer l'exploration, éxecutez les commandes suivantes depuis le catkin-workspace :
``` bash
catkin_make
source devel/setup.bash
roslaunch grp-rose challenge3_tbot.launch
```

Hors simulation le robot se comporte comme décrit dans le paragraphe précédent. Cependant, hors simulation le robot détectera aussi les bouteilles oranges placées dans l'arène grâce à la caméra RealSense et une méthode de détection par seuillage colorimétrique avec conversion HSV.

Il est possible d'écouter le topic `/bottle` pour voir la position de ces dernières :
``` bash
rostopic echo /bottle
```

Dans les deux cas, vous pouvez aussi écouter le topic `/move_base_simple/goal` pour voir les goals que vous envoyez au robot :
``` bash
rostopic echo /move_base_simple/goal
```
