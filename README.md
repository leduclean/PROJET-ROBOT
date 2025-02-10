# PROJET-ROBOT

## Présentation du projet

Ce projet de robot, réalisé en groupe dans le cadre de notre prépa, consiste en la conception d'un robot suiveur de ligne et/ou éviteur d'obstacles. L'objectif principal est de développer une solution modulaire et évolutive, facilitant ainsi la compréhension, la maintenance et l'extension du code.

## Architecture du projet

Nous avons adopté une approche orientée objet, en structurant le projet autour de classes. Cette démarche présente plusieurs avantages :

- **Modularité et séparation des responsabilités** : Chaque classe regroupe une fonctionnalité spécifique du robot (par exemple, la gestion globale du robot ou le contrôle des moteurs), ce qui rend le code plus clair et plus facile à maintenir.
- **Réutilisabilité** : En isolant les fonctionnalités dans des classes distinctes, il devient plus simple de réutiliser ou d'étendre ces composants dans d'autres projets.
- **Lisibilité** : La séparation entre les déclarations (fichiers header) et les implémentations (fichiers .cpp) permet de mieux organiser le code, facilitant ainsi la compréhension pour tous les membres de l'équipe et pour les personnes extérieures, comme notre professeure.

Voici la structure simplifiée du projet pour faciliter la compréhension :

PROJET-ROBOT-main/
    ROBOT.ino.ino
    ROBOT_CONFIG.h
    Robot.h
    Robot.cpp
    moteurs.h
    moteurs.cpp

## Description des fichiers principaux

- **ROBOT.ino.ino**  
  Fichier principal contenant le code exécuté par l'Arduino. Il initialise le système, instancie la classe `Robot` et lance la boucle principale de contrôle.

- **Robot.h / Robot.cpp**  
  Ces fichiers définissent la classe `Robot`, qui centralise la logique de gestion des capteurs, des états et la coordination générale du robot. L'approche orientée objet utilisée ici facilite l'intégration de nouvelles fonctionnalités et la modification du comportement global du robot sans impacter les autres modules.

- **moteurs.h / moteurs.cpp**  
  Ces fichiers contiennent la classe ou les fonctions dédiées au contrôle des moteurs du robot. En séparant la logique de déplacement de la gestion globale du robot, nous assurons une meilleure isolation des fonctionnalités et simplifions la maintenance.

- **ROBOT_CONFIG.h**  
  Ce fichier centralise les paramètres et constantes du robot, permettant une configuration rapide et aisée du système sans modifier directement le code source.

## Avantages de l'approche orientée objet dans ce projet

L'utilisation des classes permet d'obtenir un code plus modulaire et clair. Voici quelques bénéfices concrets de cette approche :

- **Clarté du code** : Chaque classe a une responsabilité bien définie, ce qui aide à comprendre rapidement le rôle de chaque composant.
- **Facilité de maintenance** : Les modifications ou ajouts de fonctionnalités peuvent être effectués de manière isolée dans les classes concernées, réduisant ainsi le risque d'introduire des bugs dans d'autres parties du projet.
- **Évolutivité** : Cette architecture permet d'étendre facilement le projet en ajoutant de nouvelles classes ou en modifiant les existantes, sans avoir à réécrire l'ensemble du code.


## Auteurs

Ce projet a été réalisé en groupe dans le cadre de notre prépa par Léandre LE DUC, Garance BARRET, Dimitri BEOIR, Ilan VARGAS.
