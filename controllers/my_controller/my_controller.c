#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 64

// Fonctions pour les ensembles flous
double triangle(double x, double a, double b, double c) {
    if (x <= a || x >= c) return 0.0;
    if (x == b) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

double trapeze(double x, double a, double b, double c, double d) {
    if (x <= a || x >= d) return 0.0;
    if (x >= b && x <= c) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

int main() {
    wb_robot_init();

    // Récupération des capteurs
    WbDeviceTag ps1 = wb_robot_get_device("ps1");
    WbDeviceTag ps2 = wb_robot_get_device("ps2");
    WbDeviceTag ps5 = wb_robot_get_device("ps5");
    WbDeviceTag ps6 = wb_robot_get_device("ps6");

    wb_distance_sensor_enable(ps1, TIME_STEP);
    wb_distance_sensor_enable(ps2, TIME_STEP);
    wb_distance_sensor_enable(ps5, TIME_STEP);
    wb_distance_sensor_enable(ps6, TIME_STEP);

    // Récupération des moteurs
    WbDeviceTag motor_gauche = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_droit  = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(motor_gauche, INFINITY);
    wb_motor_set_position(motor_droit, INFINITY);

    while (wb_robot_step(TIME_STEP) != -1) {
        
        // Lecture des capteurs
        double v_ps1 = wb_distance_sensor_get_value(ps1);
        double v_ps2 = wb_distance_sensor_get_value(ps2);
        double v_ps5 = wb_distance_sensor_get_value(ps5);
        double v_ps6 = wb_distance_sensor_get_value(ps6);

        // On garde la valeur max de chaque côté
        double distance_droite = fmax(v_ps1, v_ps2);
        double distance_gauche = fmax(v_ps5, v_ps6);

        // Fuzzification (calcul des degrés d'appartenance)
        double g_loin   = trapeze(distance_gauche,    0,    0,   30,   80);
        double g_moyen  = triangle(distance_gauche,  30,   80,  150);
        double g_proche = trapeze(distance_gauche,  100,  200, 4096, 4096);
          
        double d_loin   = trapeze(distance_droite,    0,    0,   30,   80);
        double d_moyen  = triangle(distance_droite,  30,   80,  150);
        double d_proche = trapeze(distance_droite,  100,  200, 4096, 4096);

        // Affichage pour le debug
        printf("G: %6.0f -> loin=%.2f moyen=%.2f proche=%.2f | "
               "D: %6.0f -> loin=%.2f moyen=%.2f proche=%.2f\n",
               distance_gauche, g_loin, g_moyen, g_proche,
               distance_droite, d_loin, d_moyen, d_proche);

        // Règles d'inférence
        double r1_orient = g_proche;               
        double r2_orient = d_proche;               
        double r3_orient = fmin(g_loin, d_loin);   

        double r1_vit = fmax(g_proche, d_proche);  
        double r2_vit = fmax(g_moyen, d_moyen);    
        double r3_vit = fmin(g_loin, d_loin);      

        // Défuzzification pour l'orientation (centre de gravité)
        double num_orient = 0.0;
        double den_orient = 0.0;

        for (double angle = -0.26; angle <= 0.26; angle += 0.01) {
            double mu_tg  = triangle(angle, -0.26, -0.13, 0.0);  
            double mu_td  = triangle(angle, -0.13,  0.0,  0.13); 
            double mu_tr  = triangle(angle,  0.0,   0.13, 0.26); 

            double mu_agg = 0.0;
            mu_agg = fmax(mu_agg, fmin(r1_orient, mu_tr));   
            mu_agg = fmax(mu_agg, fmin(r2_orient, mu_tg));   
            mu_agg = fmax(mu_agg, fmin(r3_orient, mu_td));   

            num_orient += angle * mu_agg;
            den_orient += mu_agg;
        }

        double orientation = (den_orient > 1e-6) ? (num_orient / den_orient) : 0.0;

        // Défuzzification pour la vitesse
        double num_vit = 0.0;
        double den_vit = 0.0;

        for (double v = 0.0; v <= 4.0; v += 0.05) {
            double mu_lente   = trapeze(v, 0.0, 0.0, 1.0, 2.0);
            double mu_moyenne = triangle(v, 1.0, 2.0, 3.0);
            double mu_rapide  = trapeze(v, 2.5, 3.0, 4.0, 4.0);

            double mu_agg = 0.0;
            mu_agg = fmax(mu_agg, fmin(r1_vit, mu_lente));
            mu_agg = fmax(mu_agg, fmin(r2_vit, mu_moyenne));
            mu_agg = fmax(mu_agg, fmin(r3_vit, mu_rapide));

            num_vit += v * mu_agg;
            den_vit += mu_agg;
        }

        double vitesse = (den_vit > 1e-6) ? (num_vit / den_vit) : 1.0;

        // Conversion en vitesse des roues
        double K = 1.5; 
        double vg = vitesse + K * orientation;
        double vd = vitesse - K * orientation;

        // Limites de sécurité
        if (vg > 6.28) vg = 6.28;
        if (vg < -6.28) vg = -6.28;
        if (vd > 6.28) vd = 6.28;
        if (vd < -6.28) vd = -6.28;

        printf("ORIENT=%.2f rad | VITESSE=%.2f | VG=%.2f VD=%.2f\n",
               orientation, vitesse, vg, vd);

        // Envoi des vitesses aux moteurs
        wb_motor_set_velocity(motor_gauche, vg);
        wb_motor_set_velocity(motor_droit, vd);
    }

    wb_robot_cleanup();
    return 0;
}