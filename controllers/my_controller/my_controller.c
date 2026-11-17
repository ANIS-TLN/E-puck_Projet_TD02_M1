#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>

#define TIME_STEP 64

int main(int argc, char **argv)
{
    // 1) Initialisation du robot
    wb_robot_init();

    // 2) Récupération du pas de temps
    int timestep = (int)wb_robot_get_basic_time_step();

    // 3) Récupération et activation des capteurs
    WbDeviceTag capteurs[8];

    const char *noms_capteurs[8] = {
        "ps0", "ps1", "ps2", "ps3",
        "ps4", "ps5", "ps6", "ps7"
    };

    for (int i = 0; i < 8; i++) {
        capteurs[i] = wb_robot_get_device(noms_capteurs[i]);
        wb_distance_sensor_enable(capteurs[i], timestep);
    }

    // 4) Récupération des moteurs
    WbDeviceTag moteur_gauche =
        wb_robot_get_device("left wheel motor");

    WbDeviceTag moteur_droit =
        wb_robot_get_device("right wheel motor");

    // 5) Passage des moteurs en mode vitesse
    wb_motor_set_position(moteur_gauche, INFINITY);
    wb_motor_set_position(moteur_droit, INFINITY);

    // Arrêt initial
    wb_motor_set_velocity(moteur_gauche, 0.0);
    wb_motor_set_velocity(moteur_droit, 0.0);

    // 6) Boucle principale
    while (wb_robot_step(timestep) != -1) {

        // Lecture des capteurs
        double valeurs[8];

        for (int i = 0; i < 8; i++) {
            valeurs[i] = wb_distance_sensor_get_value(capteurs[i]);
        }

        // Capteurs du côté droit
        double ps1 = valeurs[1];
        double ps2 = valeurs[2];

        // Capteurs du côté gauche
        double ps5 = valeurs[5];
        double ps6 = valeurs[6];

        // 7) Logique de commande (à compléter)
        double vitesse_gauche = 2.0;
        double vitesse_droite = 2.0;

        // 8) Commande des moteurs
        wb_motor_set_velocity(moteur_gauche, vitesse_gauche);
        wb_motor_set_velocity(moteur_droit, vitesse_droite);
    }

    // 9) Arrêt du robot
    wb_robot_cleanup();

    return EXIT_SUCCESS;
}
