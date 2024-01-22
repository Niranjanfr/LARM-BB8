# LARM_BB8

## Description

En utilisant les instructions et le code fournis ci-dessous, vous pourrez permettre au robot ```Kobuki turtleBot3``` de se déplacer de manière autonome tout en évitant les obstacles. De manière additionnelle, le robot est capable de cartographier l'environnement qui l'entoure et de détecter les bouteilles verte de NukaCola.

Ce répertoire contient :

1 package :

1. grpe_larm_mother        (les fichiers launch du challenge 1 & 2)

## Installation

On utilise ubuntu 20.04 et on suppose les bibliothèques suivantes installées : 
 ```
pip3 install numpy tensorflow opencv-python opencv-contrib-python scikit-learn scipy matplotlib psutil scikit-image
```
 avec le workspace `~/ros2_ws/` crée.


**Etape 1 :**

Clonez le package suivant :

```
cd ~/ros2_ws/

git clone https://github.com/Niranjanfr/LARM-BB8.git
```


**Etape 2 :**

Build et Sourcer le workspace :
```
cd ~/ros2_ws/
colcon build
source ./install/setup.bash
```

**Etape 3 :**

## Exécution

**Lancement depuis l'appareil opérateur :**

Se connecter en ssh pour lancer les launch files nécessaire avec les commandes suivantes : 

```
ip a
```
sur la machine pilote, pour recupérer l'addresse IP, puis
```
ssh nom_machine@adresse_ip
```
sur la machine opérateur pour controler le pilote à distance. Et enfin :
```
export ROS_DOMAIN_ID = xx
```
en rentrant l'addresse subnet sur laquelle la machine pilote se situe.

**Robot :**
 
Pour lancer le robot avec detection des obstacles, executer la commande suivante:
```
ros2 run grpe_larm_mother reactive_move.py
```

**Vision :**

Le traitement pour détecter les bouteilles se fait via un filtre de couleur en HSV.
On effectue également une selection selon la taille de l'objet et la distance de l'objet par rapport au robot avec un travail sur la perspective pour éliminer les objets trop grand car trop devant par rapport à une vraie bouteille. La selection par la taille se fait via une restriction du rayon des cercles circonscrits aux objets.
Nous avons tenter de faire un selection par une ellipse (car plus restrictif) en travaillant sur le rapport demi-grand-axe/demi-petit-axe constant selon la distance de l'objet et "unique" pour l'objet. Par soucis de manque de temps la solution non débuggé est dans le fichier en commenté avec "###" indiqué devant la ligne.

Nous avons également prévu le cas particulier dans lequel le robot ferait face à deux bouteilles en même temps : les deux sont détectés puis ajouté sur la map.


Pour visualiser la caméra seulement, exécuter le noeud suivant sur la machine pilote en décommantant les lignes indiqués dans le code:
```
ros2 run grpe_larm_mother camera_driver.py
```

**Challenge 1 :**

Dans ce premier challenge, le robot se déplace à l'intérieur d'un environnement clos en évitant tous les obstacles sur son chemin.

Pour effectuer la connexion avec le robot, executer la commande suivante :
```
ros2 launch grpe_larm_mother minimal_launch.yaml
```

Pour lancer le robot avec detection des obstacles et visualisation, exécutez la commande suivante :
```
ros2 launch grpe_larm_mother challenge1.yaml
```

Pour lancer la simulation :
```
ros2 launch grpe_larm_mother simulation.yaml
```
**Challenge 2 :**

Dans ce deuxième challenge, on ajoute au premier challenge, le SLAM ainsi que le positionnement unicitaire des objets sur la map.

pour lancer le challenge 2 avec : connexion au robot + deplacement automatique + camera + SLAM + marqueur des objets, exécuter la commande suivante:
```
ros2 launch grpe_larm_mother challenge2.yaml
```

pour lancer Rviz, exécuter la commande suivante : 
```
ros2 launch grpe_larm_mother rviz.yaml
```
ou 
```
rviz2
```

Pour lancer la simulation :
```
ros2 launch grpe_larm_mother simulation2.yaml
```

