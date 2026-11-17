from controller import Robot

# 1) Création de l'objet Robot
robot = Robot()

# 2) Récupération du pas de temps
timestep = int(robot.getBasicTimeStep())

# 3) Récupération et activation des capteurs de distance
capteurs = []

noms_capteurs = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for nom in noms_capteurs:
    capteur = robot.getDevice(nom)
    capteur.enable(timestep)
    capteurs.append(capteur)

# 4) Récupération des moteurs des deux roues
moteur_gauche = robot.getDevice('left wheel motor')
moteur_droit = robot.getDevice('right wheel motor')

# Passage des moteurs en mode vitesse
moteur_gauche.setPosition(float('inf'))
moteur_droit.setPosition(float('inf'))

# Arrêt initial des moteurs
moteur_gauche.setVelocity(0.0)
moteur_droit.setVelocity(0.0)

# 5) Boucle principale
while robot.step(timestep) != -1:

    # Lecture des capteurs
    valeurs = [c.getValue() for c in capteurs]

    # Capteurs du côté droit
    ps1, ps2 = valeurs[1], valeurs[2]

    # Capteurs du côté gauche
    ps5, ps6 = valeurs[5], valeurs[6]

    # 6) Logique de commande (à compléter)
    vitesse_gauche = 2.0
    vitesse_droite = 2.0

    # 7) Commande des moteurs
    moteur_gauche.setVelocity(vitesse_gauche)
    moteur_droit.setVelocity(vitesse_droite)
