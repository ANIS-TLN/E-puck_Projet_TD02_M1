#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 64

// ============================================================
// Fonctions d'appartenance (AVANT main())
// ============================================================
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

// ============================================================
// Main
// ============================================================
int main() {
    wb_robot_init();

    WbDeviceTag ps1 = wb_robot_get_device("ps1");
    WbDeviceTag ps2 = wb_robot_get_device("ps2");
    WbDeviceTag ps5 = wb_robot_get_device("ps5");
    WbDeviceTag ps6 = wb_robot_get_device("ps6");

    wb_distance_sensor_enable(ps1, TIME_STEP);
    wb_distance_sensor_enable(ps2, TIME_STEP);
    wb_distance_sensor_enable(ps5, TIME_STEP);
    wb_distance_sensor_enable(ps6, TIME_STEP);

    WbDeviceTag motor_gauche = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_droit  = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(motor_gauche, INFINITY);
    wb_motor_set_position(motor_droit, INFINITY);

    // ---- Boucle principale ----
    while (wb_robot_step(TIME_STEP) != -1) {
        // 1) Lecture des capteurs
        double v_ps1 = wb_distance_sensor_get_value(ps1);
        double v_ps2 = wb_distance_sensor_get_value(ps2);
        double v_ps5 = wb_distance_sensor_get_value(ps5);
        double v_ps6 = wb_distance_sensor_get_value(ps6);

        // 2) Fusion : max de chaque côté
        double distance_droite = fmax(v_ps1, v_ps2);
        double distance_gauche = fmax(v_ps5, v_ps6);

        // 3) Fuzzification côté GAUCHE
        double g_loin   = trapeze(distance_gauche,    0,    0,  800, 1800);
        double g_moyen  = triangle(distance_gauche, 1000, 2000, 3000);
        double g_proche = trapeze(distance_gauche,  2500, 3200, 4096, 4096);

        // 4) Fuzzification côté DROIT
        double d_loin   = trapeze(distance_droite,    0,    0,  800, 1800);
        double d_moyen  = triangle(distance_droite, 1000, 2000, 3000);
        double d_proche = trapeze(distance_droite,  2500, 3200, 4096, 4096);

        // 5) Debug fuzzification
        printf("G: %6.0f -> loin=%.2f moyen=%.2f proche=%.2f | "
               "D: %6.0f -> loin=%.2f moyen=%.2f proche=%.2f\n",
               distance_gauche, g_loin, g_moyen, g_proche,
               distance_droite, d_loin, d_moyen, d_proche);

        // ============================================================
        // 6) RÈGLES D'INFÉRENCE (base corrigée, 3 règles par sortie)
        // ============================================================

        // --- Orientation ---
        // R1 : Gauche Proche -> Tourner_Droite
        double r1_orient = g_proche;
        // R2 : Droite Proche -> Tourner_Gauche
        double r2_orient = d_proche;
        // R3 : Gauche Loin ET Droite Loin -> Tout_Droit
        double r3_orient = fmin(g_loin, d_loin);

        // --- Vitesse ---
        // R1 : Gauche Proche OU Droite Proche -> Lente
        double r1_vit = fmax(g_proche, d_proche);
        // R2 : Gauche Moyen OU Droite Moyen -> Moyenne
        double r2_vit = fmax(g_moyen, d_moyen);
        // R3 : Gauche Loin ET Droite Loin -> Rapide
        double r3_vit = fmin(g_loin, d_loin);

        // ============================================================
        // 7) DÉFUZZIFICATION : orientation (centroïde)
        // ============================================================
        double num_orient = 0.0;
        double den_orient = 0.0;

        for (double angle = -1.2; angle <= 1.2; angle += 0.01) {
            double mu_tg  = triangle(angle, -0.6, -0.3,  0.0);   // Tourner_Gauche
            double mu_td  = triangle(angle, -0.15, 0.0,  0.15);  // Tout_Droit
            double mu_tr  = triangle(angle,  0.0,  0.3,  0.6);   // Tourner_Droite

            // Agrégation Mamdani (max des min)
            double mu_agg = 0.0;
            mu_agg = fmax(mu_agg, fmin(r1_orient, mu_tr));   // R1 -> Tourner_Droite
            mu_agg = fmax(mu_agg, fmin(r2_orient, mu_tg));   // R2 -> Tourner_Gauche
            mu_agg = fmax(mu_agg, fmin(r3_orient, mu_td));   // R3 -> Tout_Droit

            num_orient += angle * mu_agg;
            den_orient += mu_agg;
        }

        double orientation;
        if (den_orient > 1e-6) {
            orientation = num_orient / den_orient;
        } else {
            orientation = 0.0;
        }

        // ============================================================
        // 8) DÉFUZZIFICATION : vitesse (centroïde)
        // ============================================================
        double num_vit = 0.0;
        double den_vit = 0.0;

        for (double v = 0.0; v <= 6.28; v += 0.02) {
            double mu_lente   = trapeze(v, 0.0, 0.0, 1.0, 2.5);
            double mu_moyenne = triangle(v, 1.5, 3.0, 4.5);
            double mu_rapide  = trapeze(v, 3.5, 5.0, 6.28, 6.28);

            double mu_agg = 0.0;
            mu_agg = fmax(mu_agg, fmin(r1_vit, mu_lente));
            mu_agg = fmax(mu_agg, fmin(r2_vit, mu_moyenne));
            mu_agg = fmax(mu_agg, fmin(r3_vit, mu_rapide));

            num_vit += v * mu_agg;
            den_vit += mu_agg;
        }

        double vitesse;
        if (den_vit > 1e-6) {
            vitesse = num_vit / den_vit;
        } else {
            vitesse = 1.0;
        }

        // ============================================================
        // 9) Conversion (orientation, vitesse) -> vitesses de roues
        // ============================================================
        double K = 2.0;
        double vg = vitesse + K * orientation;
        double vd = vitesse - K * orientation;

        if (vg > 6.28) vg = 6.28;
        if (vg < -6.28) vg = -6.28;
        if (vd > 6.28) vd = 6.28;
        if (vd < -6.28) vd = -6.28;

        // Debug
        printf("ORIENT=%.2f rad | VITESSE=%.2f | VG=%.2f VD=%.2f\n",
               orientation, vitesse, vg, vd);

        // 10) Commande des moteurs
        wb_motor_set_velocity(motor_gauche, vg);
        wb_motor_set_velocity(motor_droit, vd);
    }

    wb_robot_cleanup();
    return 0;
}