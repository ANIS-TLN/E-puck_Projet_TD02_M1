# Projet e-puck : Contrôle par Logique Floue (TD1 - Wargame)

**Université de Cergy-Pontoise - Master 1 I&ISC**  
**Module : Intelligence Artificielle**

## 📝 Description du projet
Ce projet consiste à concevoir et implémenter un contrôleur basé sur la **logique floue** pour un robot mobile e-puck évoluant dans l'environnement de simulation **Webots**. 
L'objectif principal est de permettre au robot de suivre un mur à une distance de sécurité, en évitant les obstacles présents dans l'arène.

## 🤖 Environnement et Robot
* **Simulateur** : Webots
* **Arène** : `RectangularArena` (1.5 x 1.5) avec des obstacles `WoodenBox` reproduisant le parcours imposé.
* **Robot** : e-puck
* **Actionneurs** : `left wheel motor` et `right wheel motor`.
* **Capteurs de distance utilisés** : `ps1`, `ps2` (droite) et `ps5`, `ps6` (gauche). La distance à l'obstacle le plus proche de chaque côté est obtenue via la valeur maximale de ces capteurs.

## 🧠 Système de Logique Floue
Le contrôleur repose sur deux entrées et deux sorties :

### Entrées (Variables linguistiques)
1. **Distance à droite** (max des capteurs `ps1`, `ps2`) : *Proche, Moyen, Loin*
2. **Distance à gauche** (max des capteurs `ps5`, `ps6`) : *Proche, Moyen, Loin*

### Sorties (Variables linguistiques)
1. **Orientation** (angle par rapport à l'orientation actuelle) : *Tourner à gauche, Tout droit, Tourner à droite*
2. **Vitesse** (dans l'orientation actuelle) : *Lente, Normale, Rapide*

### Règles floues
*(Note : à compléter avec tes propres règles si elles diffèrent)*
* **Règles d'orientation** : 3 règles principales gérant la direction à prendre en fonction des distances gauche/droite.
* **Règles de vitesse** : Ajustement de la vitesse pour éviter les chocs et maintenir un suivi fluide.

Une fonction mathématique convertit ensuite l'angle et la vitesse désirés en vitesses instantanées pour les roues gauche et droite.

## 📁 Structure du dépôt
```text
E-puck_Projet_TD02_M1/
├── controllers/
│   ├── my_controller/       # Code source du contrôleur flou (C)
│   └── pr2_demo/            # Contrôleur de démonstration
├── worlds/
│   └── TD2.wbt              # Fichier de simulation Webots
├── .gitignore
└── README.md
