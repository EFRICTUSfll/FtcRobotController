package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * ════════════════════════════════════════════════════════════
 *  MAX — TeleOp National
 *
 *  LOGIQUE DE TIR AUTOMATIQUE :
 *  ┌─────────────────────────────────────────────────────┐
 *  │  1. La Limelight détecte un AprilTag                │
 *  │  2. Le turret se centre automatiquement (PD)        │
 *  │  3. La distance est estimée via ty (trigonométrie)  │
 *  │  4. L'angle et la vitesse du shooter s'ajustent     │
 *  │  5. LED VERTE → tout est prêt                       │
 *  │  6. Tu appuies sur CARRÉ → les balles partent       │
 *  └─────────────────────────────────────────────────────┘
 *
 *  CONTRÔLES :
 *   • Joystick droit     → déplacement
 *   • Joystick gauche X  → rotation
 *   • DPAD ↑/↓           → vitesse de déplacement
 *   • L2 / R2            → turret manuel (override)
 *   • Rond               → cycle preset distance manuel
 *   • Triangle           → toggle shooter ON/OFF manuel
 *   • CARRÉ              → TIRER (envoyer les balles)
 *   • L1                 → toggle ramassage
 *   • X                  → toggle montage
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "calibration Shooter", group = "CALIBRATION")
public class shooterFin extends LinearOpMode {

    // ════════════════════════════════════════════════════════
    //  HARDWARE
    // ════════════════════════════════════════════════════════

    public Servo      light = null;

    DcMotor           moteurAvantGauche;
    DcMotor           moteurAvantDroit;
    DcMotor           moteurArriereGauche;
    DcMotor           moteurArriereDroit;
    DcMotorEx         shooter;
    DcMotor           intakeMoteur;
    DcMotor           montageGauche;
    DcMotor           montageDroit;

    CRServo           servoRamassageDroit;
    CRServo           servoRamassageGauche;
    CRServo           servoMoteurRamassageBalle;
    CRServo           servoTurret;
    Servo             servoAngleShooter;

    IMU               imu;
    Limelight3A       limelight;
    GoBildaPinpointDriver pinpoint;

    // ════════════════════════════════════════════════════════
    //  PIDF SHOOTER
    //  GoBilda Yellow Jacket 6000 RPM
    //  Unités : ticks/seconde (encoder intégré)
    // ════════════════════════════════════════════════════════

    private static final double SHOOTER_P         = 15.0;
    private static final double SHOOTER_I         =  5.1;
    private static final double SHOOTER_D         =  0.5;
    private static final double SHOOTER_F         = 10.5;

    /** Écart de vitesse (ticks/s) acceptable pour considérer le shooter "prêt" */
    private static final double SHOOTER_TOLERANCE = 30.0;

    // ════════════════════════════════════════════════════════
    //  CAMÉRA LIMELIGHT — PARAMÈTRES DE DISTANCE
    //
    //  On utilise ty (angle vertical) pour estimer la distance
    //  au tag via trigonométrie :
    //
    //  distance_cm = (HAUTEUR_TAG_CM - HAUTEUR_CAMERA_CM)
    //                / tan( ANGLE_CAMERA_DEG + ty )
    //
    //  ⚠️ À AJUSTER selon ton robot :
    // ════════════════════════════════════════════════════════

    /**
     * Angle d'inclinaison de la Limelight vers le bas (degrés).
     * Tu as mesuré entre 32° et 33° → on prend 32.5°.
     * Ajuste si la distance calculée est fausse.
     */
    private static final double ANGLE_CAMERA_DEG  = 32.5;

    /**
     * Hauteur du centre de la lentille de la Limelight depuis le sol (cm).
     * ⚠️ MESURE CE CHIFFRE SUR TON ROBOT et remplace 30.0 par ta valeur.
     */
    private static final double HAUTEUR_CAMERA_CM = 30.0;

    /**
     * Hauteur du centre de l'AprilTag cible depuis le sol (cm).
     * ⚠️ MESURE LA HAUTEUR DU TAG sur le terrain et ajuste.
     * Exemple : tag au centre à 60 cm du sol → 60.0
     */
    private static final double HAUTEUR_TAG_CM    = 60.0;

    /**
     * Distance minimale prise en compte (cm).
     * En dessous, on ignore la mesure car trop imprécise.
     */
    private static final double DISTANCE_MIN_CM   = 10.0;

    /**
     * Distance maximale prise en compte (cm).
     * Au-delà, on ignore et on garde le dernier preset valide.
     */
    private static final double DISTANCE_MAX_CM   = 250.0;

    // ════════════════════════════════════════════════════════
    //  TABLE DE TIR — PRESETS DISTANCE → ANGLE + VITESSE
    //
    //  Structure : { distanceCm, angleServo (0.0–1.0), vitesseTicks/s }
    //
    //  angleServo :
    //    0.0 = 0°   (shooter à plat)
    //    1.0 = 300° (servo GoBilda mode ANGLE)
    //    → Plus la distance est grande, plus l'angle doit être élevé
    //      et la vitesse plus forte.
    //
    //  ⚠️ CES VALEURS SONT DES ESTIMATIONS DE TEST.
    //  ⚠️ AJUSTE chaque ligne après tes tirs réels.
    //  ⚠️ Tu peux ajouter / supprimer des lignes librement.
    //     Le code interpolera automatiquement entre les points.
    //
    //  Colonnes : { distance (cm), angleServo, vitesse (ticks/s) }
    // ════════════════════════════════════════════════════════

    private static final double[][] TABLE_TIR = {
            //  dist(cm)  angle   ticks/s
            {   10.0,     0.95,    800.0  },   // très proche
            {   30.0,     0.88,   1000.0  },   // ~30 cm
            {   50.0,     0.80,   1200.0  },   // ~50 cm
            {   70.0,     0.72,   1500.0  },   // ~70 cm
            {   90.0,     0.63,   1800.0  },   // ~90 cm
            {  110.0,     0.55,   2100.0  },   // ~110 cm
            {  130.0,     0.47,   2400.0  },   // ~130 cm
            {  150.0,     0.38,   2750.0  },   // ~150 cm
            {  175.0,     0.28,   3100.0  },   // ~175 cm
            {  200.0,     0.18,   3500.0  },   // ~200 cm
            {  250.0,     0.10,   3800.0  },   // ~250 cm (limite max)
    };

    // ════════════════════════════════════════════════════════
    //  PRESETS MANUELS (bouton Rond)
    //  Utilisés quand aucun tag n'est détecté.
    //  Même structure que TABLE_TIR mais sans la colonne distance.
    //  { angleServo, vitesse ticks/s }
    // ════════════════════════════════════════════════════════

    private static final double[][] PRESETS_MANUEL = {
            { 0.95,  800.0  },   // Preset 1  — ~10  cm
            { 0.88, 1000.0  },   // Preset 2  — ~30  cm
            { 0.80, 1200.0  },   // Preset 3  — ~50  cm
            { 0.72, 1500.0  },   // Preset 4  — ~70  cm
            { 0.63, 1800.0  },   // Preset 5  — ~90  cm
            { 0.55, 2100.0  },   // Preset 6  — ~110 cm
            { 0.47, 2400.0  },   // Preset 7  — ~130 cm
            { 0.38, 2750.0  },   // Preset 8  — ~150 cm
            { 0.28, 3100.0  },   // Preset 9  — ~175 cm
            { 0.18, 3500.0  },   // Preset 10 — ~200 cm
    };

    private int presetManuelActif = 0;

    // ════════════════════════════════════════════════════════
    //  TURRET — SUIVI APRILTAG (PD)
    // ════════════════════════════════════════════════════════

    /** -1 = suit n'importe quel tag visible */
    private static final int    TURRET_TAG_ID       = -1;

    /** Gain proportionnel turret */
    private static final double KP_TURRET           = 0.028;

    /** Gain dérivé turret — amortit les oscillations */
    private static final double KD_TURRET           = 0.004;

    /** Zone morte : sous ce seuil (degrés), on ne corrige pas */
    private static final double TURRET_DEADBAND_DEG = 2.0;

    /**
     * Zone morte "prêt à tirer" : si |tx| < cette valeur,
     * on considère que le turret est bien centré sur le tag.
     * ⚠️ Réduis si tu veux plus de précision, augmente si ça tremble.
     */
    private static final double TURRET_PRET_DEG     = 3.5;

    /** Puissance minimale pour vaincre les frottements mécaniques */
    private static final double TURRET_MIN_POWER    = 0.07;

    /** Puissance maximale turret */
    private static final double TURRET_MAX_POWER    = 0.65;

    // État interne PD turret
    private double derniereTxTurret = 0.0;
    private ElapsedTime tempsTurret = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  ÉTAT DU SYSTÈME DE TIR AUTOMATIQUE
    // ════════════════════════════════════════════════════════

    /** true si un AprilTag est actuellement visible et valide */
    private boolean tagVisible      = false;

    /** true si le turret est centré sur le tag (|tx| < TURRET_PRET_DEG) */
    private boolean turretCentre    = false;

    /** true si le shooter a atteint sa vitesse cible */
    private boolean shooterPret     = false;

    /**
     * Vitesse cible actuelle du shooter (ticks/s).
     * Mise à jour automatiquement par calcul de distance si tag visible,
     * ou par preset manuel sinon.
     */
    private double vitesseCibleShooter = 0;

    /**
     * Angle cible actuel du servo shooter (0.0–1.0).
     * Mis à jour de la même façon que vitesseCibleShooter.
     */
    private double angleCibleShooter   = PRESETS_MANUEL[0][0];

    /** Distance estimée au tag (cm), -1 si pas de tag */
    private double distanceEstimee     = -1;

    // ════════════════════════════════════════════════════════
    //  ODOMÉTRIE — STABILISATION À L'ARRÊT (Pinpoint)
    // ════════════════════════════════════════════════════════

    private static final double KP_STABILISATION    = 0.4;
    private static final double SEUIL_ARRET_MMS     = 15.0;
    private static final double MAX_CORRECTION_STAB = 0.20;

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT — RAMPE ACC/DEC
    // ════════════════════════════════════════════════════════

    private double vitesseDeplacement             = 0.7;
    private double puissanceAvantGaucheActuelle   = 0;
    private double puissanceAvantDroitActuelle    = 0;
    private double puissanceArriereGaucheActuelle = 0;
    private double puissanceArriereDroitActuelle  = 0;

    private static final double TAUX_ACCELERATION = 0.25;
    private static final double TAUX_DECELERATION = 0.35;
    private static final double SEUIL_MORT        = 0.05;

    // ════════════════════════════════════════════════════════
    //  ÉTATS BOOLÉENS
    // ════════════════════════════════════════════════════════

    private boolean shooterActif      = false;
    private boolean estRamassageActif = false;
    private boolean montageActif      = false;

    // ════════════════════════════════════════════════════════
    //  TIMERS DEBOUNCE
    // ════════════════════════════════════════════════════════

    private ElapsedTime tempsLoop                 = new ElapsedTime();
    private ElapsedTime tempsDebounceVitesse      = new ElapsedTime();
    private ElapsedTime tempsDebounceRamassage    = new ElapsedTime();
    private ElapsedTime tempsDebounceShooter      = new ElapsedTime();
    private ElapsedTime tempsDebonceMontage       = new ElapsedTime();
    private ElapsedTime tempsDebounceAngleShooter = new ElapsedTime();
    private ElapsedTime tempsDebonceTir           = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  BOUCLE PRINCIPALE
    // ════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        configurerPIDShooter();

        light.setPosition(0.160); // LED bleue pendant l'init
        waitForStart();
        tempsLoop.reset();
        tempsTurret.reset();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacementFluide();
            gestionTurretEtDistance();  // turret + calcul distance + mise à jour cibles shooter
            gestionShooterAuto();       // shooter tourne si tag visible
            gestionTir();               // CARRÉ → envoie les balles si LED verte
            gestionRamassage();
            gestionMontage();
            gestionAngleShooter();
            gestionLED();
            afficherTelemetry();

            telemetry.update();
        }

        arretMoteurs();
    }

    // ════════════════════════════════════════════════════════
    //  LED — INDICATEUR D'ÉTAT
    //
    //  Vert  (0.500) → Tag visible + turret centré + shooter prêt
    //                  → TU PEUX TIRER (appuie sur CARRÉ)
    //  Orange(0.388) → Shooter actif mais pas encore prêt / pas de tag
    //  Bleu  (0.160) → Shooter inactif / en attente
    //
    //  ⚠️ Les valeurs de position correspondent aux couleurs
    //     de ta LED GoBilda. Ajuste si les couleurs sont fausses.
    // ════════════════════════════════════════════════════════

    private void gestionLED() {
        // Tous les critères sont réunis → vert = prêt à tirer
        if (tagVisible && turretCentre && shooterPret) {
            light.setPosition(0.500); // VERT
        }
        // Shooter actif mais en montée de vitesse, ou tag pas centré
        else if (shooterActif) {
            light.setPosition(0.388); // ORANGE
        }
        // Rien d'actif
        else {
            light.setPosition(0.160); // BLEU
        }
    }

    // ════════════════════════════════════════════════════════
    //  TURRET + CALCUL DE DISTANCE
    //
    //  Cette fonction fait deux choses en même temps :
    //  1. Asservit le turret sur le tag (PD)
    //  2. Calcule la distance au tag via ty et met à jour
    //     les cibles angle/vitesse du shooter
    // ════════════════════════════════════════════════════════

    private void gestionTurretEtDistance() {
        float gachetteGauche = gamepad1.left_trigger;
        float gachetteDroit  = gamepad1.right_trigger;

        // ── Mode manuel turret (L2/R2) ────────────────────
        if (gachetteDroit > 0.1 || gachetteGauche > 0.1) {
            double puissance = 0;
            if (gachetteDroit  > 0.1) puissance =  gachetteDroit  * 0.5;
            if (gachetteGauche > 0.1) puissance = -gachetteGauche * 0.5;
            servoTurret.setPower(puissance);
            derniereTxTurret = 0;
            tagVisible    = false;
            turretCentre  = false;
            distanceEstimee = -1;
            return;
        }

        // ── Lecture Limelight ─────────────────────────────
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getStaleness() > 300) {
            // Pas de résultat frais → on arrête le turret
            servoTurret.setPower(0);
            tagVisible    = false;
            turretCentre  = false;
            distanceEstimee = -1;
            // On garde les cibles du dernier preset manuel
            appliquerPresetManuel();
            return;
        }

        FiducialResult tag = trouverTag(result);

        if (tag == null) {
            servoTurret.setPower(0);
            tagVisible    = false;
            turretCentre  = false;
            distanceEstimee = -1;
            appliquerPresetManuel();
            return;
        }

        // ── Tag trouvé ────────────────────────────────────
        tagVisible = true;

        double tx = tag.getTargetXDegrees(); // angle horizontal
        double ty = tag.getTargetYDegrees(); // angle vertical (pour la distance)

        // ── Calcul de distance par trigonométrie ──────────
        // Formule : d = (H_tag - H_camera) / tan(angle_camera + ty)
        // ty est négatif quand le tag est en dessous du centre de l'image
        // (ce qui est normal avec une caméra inclinée vers le bas)
        double angleTotal_rad = Math.toRadians(ANGLE_CAMERA_DEG + ty);

        if (Math.abs(angleTotal_rad) > 0.01) { // éviter division par zéro
            double distanceCm = (HAUTEUR_TAG_CM - HAUTEUR_CAMERA_CM) / Math.tan(angleTotal_rad);

            if (distanceCm >= DISTANCE_MIN_CM && distanceCm <= DISTANCE_MAX_CM) {
                distanceEstimee = distanceCm;
                // Met à jour les cibles shooter par interpolation dans la table
                double[] cibles = interpolerTableTir(distanceCm);
                angleCibleShooter   = cibles[0];
                vitesseCibleShooter = cibles[1];
            }
            // Sinon on garde les dernières valeurs valides
        }

        // ── Asservissement turret (PD) ────────────────────
        double dt      = tempsTurret.seconds();
        double derivee = (dt > 0) ? (tx - derniereTxTurret) / dt : 0;
        derniereTxTurret = tx;
        tempsTurret.reset();

        double puissance = 0;

        if (Math.abs(tx) > TURRET_DEADBAND_DEG) {
            puissance = KP_TURRET * tx + KD_TURRET * derivee;

            if (puissance > 0 && puissance < TURRET_MIN_POWER)
                puissance = TURRET_MIN_POWER;
            else if (puissance < 0 && puissance > -TURRET_MIN_POWER)
                puissance = -TURRET_MIN_POWER;

            puissance = clamp(puissance, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        servoTurret.setPower(puissance);

        // Le turret est "centré" si on est dans la zone de précision de tir
        turretCentre = (Math.abs(tx) < TURRET_PRET_DEG);

        telemetry.addData("Turret tx", "%.1f°  pwr=%.2f  centré=%s",
                tx, puissance, turretCentre ? "OUI" : "non");
        telemetry.addData("Distance estimée", distanceEstimee >= 0
                ? String.format("%.1f cm", distanceEstimee) : "—");
    }

    // ════════════════════════════════════════════════════════
    //  SHOOTER AUTOMATIQUE
    //
    //  Le shooter démarre dès qu'un AprilTag est visible.
    //  La vitesse est mise à jour en temps réel selon la distance.
    //  Si aucun tag → le shooter s'arrête (sauf si activé manuellement
    //  avec Triangle).
    // ════════════════════════════════════════════════════════

    private void gestionShooterAuto() {
        // Triangle → toggle shooter manuel (override)
        if (tempsDebounceShooter.milliseconds() > 150 && gamepad1.triangle) {
            shooterActif = !shooterActif;
            tempsDebounceShooter.reset();
        }

        // Si tag visible → shooter actif automatiquement
        if (tagVisible) {
            shooterActif = true;
        }

        if (shooterActif) {
            shooter.setVelocity(vitesseCibleShooter);

            double vitesseActuelle = shooter.getVelocity();
            double erreur = Math.abs(vitesseActuelle - vitesseCibleShooter);
            shooterPret = (erreur < SHOOTER_TOLERANCE);

            telemetry.addData("Shooter", "%.0f → %.0f t/s | %s",
                    vitesseActuelle, vitesseCibleShooter,
                    shooterPret ? "PRÊT ✓" : "montée...");
        } else {
            shooter.setVelocity(0);
            shooterPret = false;
            telemetry.addData("Shooter", "OFF");
        }
    }

    // ════════════════════════════════════════════════════════
    //  TIR — BOUTON CARRÉ
    //
    //  Conditions pour tirer :
    //   ✓ Tag visible
    //   ✓ Turret centré (|tx| < TURRET_PRET_DEG)
    //   ✓ Shooter à vitesse (erreur < SHOOTER_TOLERANCE)
    //   → LED VERTE → appuie CARRÉ
    //
    //  Si les conditions ne sont pas réunies et que tu appuies
    //  quand même → on tire quand même mais la LED est orange
    //  (tu prends le risque).
    // ════════════════════════════════════════════════════════

    private void gestionTir() {
        boolean pretATirer = tagVisible && turretCentre && shooterPret;

        if (gamepad1.square && tempsDebonceTir.milliseconds() > 300) {
            // On tire dans tous les cas si le shooter tourne
            // (LED verte = optimal, orange = sous-optimal)
            if (shooterActif) {
                // Active le mécanisme d'envoi de balle
                servoMoteurRamassageBalle.setPower(-1.0);
                // Note : la balle sera envoyée pendant que ce bouton est maintenu
                // → gestionRamassage() gérera l'arrêt quand le bouton est relâché
            }
            tempsDebonceTir.reset();
        }

        // Si le carré est relâché ET le ramassage n'est pas actif
        // → on arrête le servo d'envoi de balle
        if (!gamepad1.square && !estRamassageActif) {
            servoMoteurRamassageBalle.setPower(0.0);
        }

        telemetry.addData("Prêt à tirer", pretATirer ? "✓ OUI — Appuie CARRÉ !" : "attente...");
    }

    // ════════════════════════════════════════════════════════
    //  ANGLE SHOOTER — SERVO 360° GOBILDA MODE ANGLE
    //
    //  En mode automatique (tag visible) : l'angle est calculé
    //  par interpolation dans la TABLE_TIR.
    //  En mode manuel (pas de tag) : l'angle suit le preset
    //  sélectionné avec Rond.
    // ════════════════════════════════════════════════════════

    private void gestionAngleShooter() {
        // Rond → cycle preset manuel (utilisé quand pas de tag)
        if (tempsDebounceAngleShooter.milliseconds() > 200 && gamepad1.circle) {
            presetManuelActif = (presetManuelActif + 1) % PRESETS_MANUEL.length;
            tempsDebounceAngleShooter.reset();
            // Si pas de tag, on met à jour les cibles manuellement
            if (!tagVisible) {
                appliquerPresetManuel();
            }
        }

        // Applique l'angle cible (calculé auto ou manuel)
        servoAngleShooter.setPosition(angleCibleShooter);

        telemetry.addData("Angle Shooter", "pos=%.2f | dist=%.0f cm",
                angleCibleShooter,
                distanceEstimee >= 0 ? distanceEstimee : -1);
    }

    // ════════════════════════════════════════════════════════
    //  INTERPOLATION TABLE DE TIR
    //
    //  Cherche les deux points encadrant la distance donnée
    //  dans TABLE_TIR et interpole linéairement l'angle et
    //  la vitesse.
    //
    //  Exemple : distance = 80 cm
    //   → entre ligne {70cm, 0.72, 1500} et {90cm, 0.63, 1800}
    //   → t = (80-70)/(90-70) = 0.5
    //   → angle  = 0.72 + 0.5*(0.63-0.72) = 0.675
    //   → ticks  = 1500 + 0.5*(1800-1500) = 1650
    // ════════════════════════════════════════════════════════

    private double[] interpolerTableTir(double distanceCm) {
        // En dessous du premier point → premier preset
        if (distanceCm <= TABLE_TIR[0][0]) {
            return new double[]{ TABLE_TIR[0][1], TABLE_TIR[0][2] };
        }
        // Au-delà du dernier point → dernier preset
        int dernierIdx = TABLE_TIR.length - 1;
        if (distanceCm >= TABLE_TIR[dernierIdx][0]) {
            return new double[]{ TABLE_TIR[dernierIdx][1], TABLE_TIR[dernierIdx][2] };
        }

        // Chercher les deux points encadrants
        for (int i = 0; i < TABLE_TIR.length - 1; i++) {
            double d0 = TABLE_TIR[i][0];
            double d1 = TABLE_TIR[i + 1][0];

            if (distanceCm >= d0 && distanceCm <= d1) {
                // Ratio d'interpolation (0.0 → 1.0)
                double t = (distanceCm - d0) / (d1 - d0);

                double angle  = TABLE_TIR[i][1] + t * (TABLE_TIR[i + 1][1] - TABLE_TIR[i][1]);
                double ticks  = TABLE_TIR[i][2] + t * (TABLE_TIR[i + 1][2] - TABLE_TIR[i][2]);

                return new double[]{ angle, ticks };
            }
        }

        // Fallback sécurité
        return new double[]{ PRESETS_MANUEL[0][0], PRESETS_MANUEL[0][1] };
    }

    /** Applique le preset manuel actif aux cibles shooter */
    private void appliquerPresetManuel() {
        angleCibleShooter   = PRESETS_MANUEL[presetManuelActif][0];
        vitesseCibleShooter = PRESETS_MANUEL[presetManuelActif][1];
    }

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT FLUIDE + STABILISATION PINPOINT
    // ════════════════════════════════════════════════════════

    public void deplacementFluide() {
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotate  =  gamepad1.left_stick_x;

        forward = appliquerZoneMorte(forward);
        right   = appliquerZoneMorte(right);
        rotate  = appliquerZoneMorte(rotate);

        boolean joystickActif = (forward != 0 || right != 0 || rotate != 0);

        pinpoint.update();
        double velX = pinpoint.getVelX(DistanceUnit.MM);
        double velY = pinpoint.getVelY(DistanceUnit.MM);
        double vitesseRobot = Math.hypot(velX, velY);

        double stabForward = 0, stabRight = 0;

        if (!joystickActif && vitesseRobot > SEUIL_ARRET_MMS) {
            stabForward = clamp(-velX / 1000.0 * KP_STABILISATION,
                    -MAX_CORRECTION_STAB, MAX_CORRECTION_STAB);
            stabRight   = clamp(-velY / 1000.0 * KP_STABILISATION,
                    -MAX_CORRECTION_STAB, MAX_CORRECTION_STAB);
        }

        double fl = forward + stabForward + right + stabRight + rotate;
        double fr = forward + stabForward - right - stabRight - rotate;
        double br = forward + stabForward + right + stabRight - rotate;
        double bl = forward + stabForward - right - stabRight + rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );
        if (maxPower > 1.0) { fl /= maxPower; fr /= maxPower; bl /= maxPower; br /= maxPower; }

        puissanceAvantGaucheActuelle   = appliquerRampe(puissanceAvantGaucheActuelle,  fl);
        puissanceAvantDroitActuelle    = appliquerRampe(puissanceAvantDroitActuelle,   fr);
        puissanceArriereGaucheActuelle = appliquerRampe(puissanceArriereGaucheActuelle, bl);
        puissanceArriereDroitActuelle  = appliquerRampe(puissanceArriereDroitActuelle,  br);

        moteurAvantGauche.setPower(vitesseDeplacement  * puissanceAvantGaucheActuelle);
        moteurAvantDroit.setPower(vitesseDeplacement   * puissanceAvantDroitActuelle);
        moteurArriereGauche.setPower(vitesseDeplacement * puissanceArriereGaucheActuelle);
        moteurArriereDroit.setPower(vitesseDeplacement  * puissanceArriereDroitActuelle);
    }

    // ════════════════════════════════════════════════════════
    //  AUTRES FONCTIONS
    // ════════════════════════════════════════════════════════

    private void gestionVitesseDeplacement() {
        if (tempsDebounceVitesse.milliseconds() > 300) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                vitesseDeplacement = Math.min(vitesseDeplacement + 0.1, 1.0);
                tempsDebounceVitesse.reset();
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                vitesseDeplacement = Math.max(vitesseDeplacement - 0.1, 0.1);
                tempsDebounceVitesse.reset();
            }
        }
    }

    private void gestionRamassage() {
        if (tempsDebounceRamassage.milliseconds() > 200 && gamepad1.left_bumper) {
            estRamassageActif = !estRamassageActif;
            tempsDebounceRamassage.reset();
        }
        if (estRamassageActif) {
            intakeMoteur.setPower(1);
            servoMoteurRamassageBalle.setPower(-1.0);
        } else if (!gamepad1.square) {
            // N'écrase pas si on est en train de tirer avec CARRÉ
            intakeMoteur.setPower(0.0);
        }
    }

    private void gestionMontage() {
        if (tempsDebonceMontage.milliseconds() > 400 && gamepad1.cross) {
            montageActif = !montageActif;
            tempsDebonceMontage.reset();
        }
        if (montageActif) {
            servoRamassageGauche.setPower(1.0);
            servoRamassageDroit.setPower(-1.0);
            servoMoteurRamassageBalle.setPower(-0.5);
            montageGauche.setPower(1.0);
            montageDroit.setPower(1.0);
        } else {
            servoRamassageGauche.setPower(0.0);
            servoRamassageDroit.setPower(0.0);
            if (!estRamassageActif && !gamepad1.square) {
                servoMoteurRamassageBalle.setPower(0.0);
            }
            montageGauche.setPower(0.0);
            montageDroit.setPower(0.0);
        }
    }

    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
        shooter.setVelocity(0);
        intakeMoteur.setPower(0);
        montageDroit.setPower(0);
        montageGauche.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoRamassageDroit.setPower(0);
        servoRamassageGauche.setPower(0);
        servoAngleShooter.setPosition(PRESETS_MANUEL[0][0]);
        limelight.stop();
    }

    // ════════════════════════════════════════════════════════
    //  TELEMETRY COMPLÈTE
    // ════════════════════════════════════════════════════════

    private void afficherTelemetry() {
        telemetry.addLine("══════════════════════════════════");
        telemetry.addData("Tag visible",   tagVisible   ? "✓ OUI" : "✗ non");
        telemetry.addData("Turret centré", turretCentre ? "✓ OUI" : "✗ non");
        telemetry.addData("Shooter prêt",  shooterPret  ? "✓ OUI" : "✗ non");
        telemetry.addData("→ TIR",
                (tagVisible && turretCentre && shooterPret)
                        ? "LED VERTE — Appuie CARRÉ !"
                        : "pas encore prêt");
        telemetry.addLine("──────────────────────────────────");
        telemetry.addData("Distance",      distanceEstimee >= 0
                ? String.format("%.1f cm", distanceEstimee) : "—");
        telemetry.addData("Vitesse cible", String.format("%.0f t/s", vitesseCibleShooter));
        telemetry.addData("Angle cible",   String.format("%.2f", angleCibleShooter));
        telemetry.addData("Preset manuel", String.format("%d/10", presetManuelActif + 1));
        telemetry.addLine("──────────────────────────────────");
        telemetry.addData("Vitesse dépl.", String.format("%.0f%%", vitesseDeplacement * 100));
        telemetry.addLine("══════════════════════════════════");
    }

    // ════════════════════════════════════════════════════════
    //  INITIALISATION
    // ════════════════════════════════════════════════════════

    private void initialisationDuRobot() {
        moteurAvantDroit     = hardwareMap.get(DcMotor.class,   "avantDroit");
        moteurAvantGauche    = hardwareMap.get(DcMotor.class,   "avantGauche");
        moteurArriereDroit   = hardwareMap.get(DcMotor.class,   "dosDroit");
        moteurArriereGauche  = hardwareMap.get(DcMotor.class,   "dosGauche");
        shooter              = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMoteur         = hardwareMap.get(DcMotor.class,   "intake");
        montageGauche        = hardwareMap.get(DcMotor.class,   "montageG");
        montageDroit         = hardwareMap.get(DcMotor.class,   "montageD");

        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageDroit       = hardwareMap.get(CRServo.class, "ramassageD");
        servoRamassageGauche      = hardwareMap.get(CRServo.class, "ramassageG");
        servoTurret               = hardwareMap.get(CRServo.class, "turret");
        servoAngleShooter         = hardwareMap.get(Servo.class,   "angleShooter");
        light                     = hardwareMap.get(Servo.class,   "light");

        imu       = hardwareMap.get(IMU.class,                   "imu");
        limelight = hardwareMap.get(Limelight3A.class,           "limelight");
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intakeMoteur.setDirection(DcMotor.Direction.REVERSE);
        montageGauche.setDirection(DcMotor.Direction.FORWARD);
        montageDroit.setDirection(DcMotorSimple.Direction.REVERSE);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moteurAvantGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurAvantDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        // Cibles initiales = premier preset manuel
        appliquerPresetManuel();
        servoAngleShooter.setPosition(angleCibleShooter);
        light.setPosition(0.388);
    }

    private void configurerPIDShooter() {
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ════════════════════════════════════════════════════════
    //  UTILITAIRES
    // ════════════════════════════════════════════════════════

    private FiducialResult trouverTag(LLResult result) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        FiducialResult meilleur   = null;
        double         meilleurTx = Double.MAX_VALUE;

        for (FiducialResult f : fiducials) {
            boolean match = (TURRET_TAG_ID == -1) || (f.getFiducialId() == TURRET_TAG_ID);
            if (match && Math.abs(f.getTargetXDegrees()) < meilleurTx) {
                meilleurTx = Math.abs(f.getTargetXDegrees());
                meilleur   = f;
            }
        }
        return meilleur;
    }

    private double appliquerRampe(double actuelle, double cible) {
        if (cible == 0 && Math.abs(actuelle) < 0.05) return 0;
        double taux = (cible == 0) ? TAUX_DECELERATION : TAUX_ACCELERATION;
        return actuelle + (cible - actuelle) * taux;
    }

    private double appliquerZoneMorte(double valeur) {
        return (Math.abs(valeur) < SEUIL_MORT) ? 0 : valeur;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}