package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;

/**
 * ════════════════════════════════════════════════════════════
 *  BRICKMAKERS — Team 31563
 *  NATIONAL LYON — TeleOp Final (version fusionnée MaxF + NationalLyon)
 * ════════════════════════════════════════════════════════════
 *
 *  CHANGEMENTS PRINCIPAUX vs les deux versions précédentes :
 *
 *  1. TRACKING PERMANENT :
 *     Le turret suit le tag EN PERMANENCE, même pendant le tir.
 *     Il n'y a plus de "mode auto vs mode manuel" pour le turret.
 *     L2/R2 = override TEMPORAIRE (tant qu'on appuie). Dès qu'on
 *     relâche, le suivi reprend automatiquement.
 *
 *  2. INTERPOLATION CONTINUE (de NationalLyon) :
 *     Angle et vitesse calculés par interpolation linéaire dans
 *     la table CALIBRATION, selon la distance mesurée depuis ty.
 *     Plus de presets discrets (plus de saut brutal).
 *
 *  3. SÉCURITÉ STALENESS :
 *     Si le tag est invisible ou vieux (>300ms), le turret
 *     maintient sa dernière puissance 100ms puis s'arrête doucement.
 *     Le shooter s'éteint si le tag disparaît plus de 500ms.
 *
 *  4. SHOOTER TOUJOURS ACTIF SI TAG VISIBLE :
 *     Plus besoin de toggle Triangle pour allumer le shooter.
 *     Il se met en route dès qu'un tag est détecté, à la bonne
 *     vitesse calculée par interpolation.
 *     Triangle reste disponible pour l'éteindre manuellement.
 *
 *  CONTRÔLES :
 *    Stick droit       → avancer / reculer / strafe
 *    Stick gauche X    → rotation
 *    Dpad haut/bas     → vitesse +/-
 *    LB                → ramassage toggle
 *    X (croix)         → montage toggle
 *    Cercle            → monter les balles (si shooter PRÊT)
 *    L2 / R2           → override turret gauche / droite (temporaire)
 *    Triangle          → éteindre le shooter manuellement
 *
 *  LED :
 *    Bleu   → attente, pas de tag
 *    Jaune  → tag visible, shooter pas encore à vitesse
 *    Rouge  → shooter actif, monte en vitesse
 *    Vert   → shooter PRÊT → appuie Cercle
 *    Orange → ramassage actif
 *
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "NATIONAL LYON FINAL", group = "National")
public class NationalLyonFinal extends LinearOpMode {

    // ════════════════════════════════════════════════════════
    //  HARDWARE
    // ════════════════════════════════════════════════════════

    public Servo light;
    DcMotor moteurAvantGauche, moteurAvantDroit;
    DcMotor           moteurArriereGauche, moteurArriereDroit;
    DcMotorEx shooter;
    DcMotor           intakeMoteur;
    DcMotor           montageGauche, montageDroit;
    CRServo servoRamassageDroit, servoRamassageGauche;
    CRServo           servoMoteurRamassageBalle;
    CRServo           servoTurret;
    Servo             servoAngleShooter;
    IMU imu;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

    // ════════════════════════════════════════════════════════
    //  GÉOMÉTRIE CAMÉRA
    //  ⚙️  Vérifie avec un mètre ruban et une app niveau
    // ════════════════════════════════════════════════════════

    /** Hauteur de la Limelight depuis le sol (cm) */
    private static final double H_CAMERA     = 36.0;

    /** Hauteur du centre du tag depuis le sol (cm) */
    private static final double H_TAG        = 15.0;

    /** Inclinaison de la caméra vers le bas (degrés) */
    private static final double ANGLE_CAMERA = 17.0;

    // ════════════════════════════════════════════════════════
    //  TABLE DE CALIBRATION
    //
    //  { distanceCm, angleServo, RPM }
    //
    //  angleServo : 0.0 = 0° / 0.306 = 110° MAX (ne jamais dépasser)
    //  RPM : tours par minute du flywheel (max ~6000 sur YJ direct drive)
    //
    //  Le code interpole automatiquement entre chaque point.
    //  Hors plage → valeur de l'extrême la plus proche.
    // ════════════════════════════════════════════════════════

    private static final double[][] CALIBRATION = {
            //  { distCm,  angleServo,  RPM  }
            {  20.0,   0.30,   3000.0 },
            {  40.0,   0.27,   3200.0 },
            {  70.0,   1.24,   3500.0 },
            { 100.0,   0.21,   3600.0 },
            { 140.0,   10.17,   3900.0 },
            { 180.0,   0.12,   4000.0 },
            { 220.0,   0.07,   4500.0 },
    };

    /** Tolérance pour déclarer le shooter PRÊT (RPM) */
    private static final double SHOOTER_TOLERANCE_RPM = 50.0;

    /** GoBilda Yellow Jacket : 28 ticks/tour moteur, direct drive */
    private static final double TICKS_PAR_TOUR = 28.0;

    private double rpmEnTicks(double rpm) { return rpm * TICKS_PAR_TOUR / 60.0; }
    private double ticksEnRpm(double ticks) { return ticks * 60.0 / TICKS_PAR_TOUR; }

    // Valeurs courantes recalculées à chaque loop
    private double angleServoCourant = CALIBRATION[0][1];
    private double vitesseCourante   = CALIBRATION[0][2];
    private double distanceCourante  = 0.0;
    private boolean shooterActif     = false;

    // ════════════════════════════════════════════════════════
    //  PIDF SHOOTER
    //  ⚙️  Ordre : F d'abord, puis P, puis D, I rester à 0
    // ════════════════════════════════════════════════════════

    private static final double SHOOTER_P = 15.0;
    private static final double SHOOTER_I =  0.0;
    private static final double SHOOTER_D =  0.5;
    private static final double SHOOTER_F = 10.5;

    // ════════════════════════════════════════════════════════
    //  TURRET — TRACKING PERMANENT
    //
    //  ⚠️  CHANGEMENT CLÉ vs les deux versions précédentes :
    //  Il n'y a plus de "modeAutoTurret". Le turret suit TOUJOURS
    //  le tag. L2/R2 = override temporaire (tant qu'on appuie).
    //
    //  TX_OFFSET_DEG : ajuste si le turret se stabilise décalé.
    //    Place un tag en face, laisse le turret se stabiliser,
    //    lis "tx corrigé" sur DS quand il est arrêté.
    //    Cette valeur = TX_OFFSET_DEG.
    // ════════════════════════════════════════════════════════

    private static final double TX_OFFSET_DEG    =  0.0;  // ← ajuste si décentré
    private static final double KP_TURRET        =  0.025;
    private static final double KD_TURRET        =  0.003;
    private static final double DEADBAND_TURRET  =  2.5;
    private static final double MIN_POWER_TURRET =  0.07;
    private static final double MAX_POWER_TURRET =  0.60;

    // ← MODIF : plus de booléen modeAutoTurret, le suivi est toujours actif
    private double      dernierTx        = 0.0;
    private ElapsedTime tempsTurret      = new ElapsedTime();
    private boolean     tagVisible       = false;

    // ← MODIF : timer pour gérer la perte de tag (inertie turret)
    private ElapsedTime tempsDepuisTag   = new ElapsedTime();
    private double      dernierePuissanceTurret = 0.0;

    // ← MODIF : timer pour éteindre le shooter si tag perdu longtemps
    private static final long SHOOTER_TAG_TIMEOUT_MS = 500;

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT
    // ════════════════════════════════════════════════════════

    private double vitesseDeplacement             = 0.7;
    private double puissanceAvantGaucheActuelle   = 0;
    private double puissanceAvantDroitActuelle    = 0;
    private double puissanceArriereGaucheActuelle = 0;
    private double puissanceArriereDroitActuelle  = 0;

    private static final double TAUX_ACCELERATION = 0.25;
    private static final double TAUX_DECELERATION = 0.35;
    private static final double SEUIL_MORT        = 0.05;
    private static final double KP_STAB           = 0.35;
    private static final double SEUIL_STAB        = 20.0;
    private static final double MAX_STAB          = 0.18;

    // ════════════════════════════════════════════════════════
    //  LED
    // ════════════════════════════════════════════════════════

    private static final double LED_OFF    = 0.388;
    private static final double LED_VERT   = 0.500;
    private static final double LED_ROUGE  = 0.160;
    private static final double LED_ORANGE = 0.222;
    private static final double LED_BLEU   = 0.611;
    private static final double LED_JAUNE  = 0.118;

    // ════════════════════════════════════════════════════════
    //  ÉTATS & TIMERS
    // ════════════════════════════════════════════════════════

    private boolean     estRamassageActif = false;
    private boolean     montageActif      = false;

    private ElapsedTime debounceVitesse   = new ElapsedTime();
    private ElapsedTime debounceRamassage = new ElapsedTime();
    private ElapsedTime debounceShooter   = new ElapsedTime();
    private ElapsedTime debounceMontage   = new ElapsedTime();
    private ElapsedTime debounceMonter    = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  BOUCLE PRINCIPALE
    // ════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        initHardware();
        configurerPID();

        light.setPosition(LED_BLEU);
        telemetry.addLine("Brickmakers 31563 — National Lyon Final");
        telemetry.addLine("Turret : suivi permanent | L2/R2 = override temporaire");
        telemetry.update();

        waitForStart();
        tempsTurret.reset();
        tempsDepuisTag.reset();

        while (opModeIsActive()) {
            // ── Déplacement (toujours manuel) ─────────────────
            gestionVitesse();
            deplacementFluide();

            // ── Turret + Distance + Tir (une seule lecture Limelight) ──
            // ← MODIF : toute la logique dans une seule méthode,
            //   le tracking est permanent, L2/R2 = override temporaire
            gestionTurretEtTir();

            // ── Shooter ───────────────────────────────────────
            gestionShooter();

            // ── Monter les balles ──────────────────────────────
            gestionMonterBalles();

            // ── Ramassage & Montage ───────────────────────────
            gestionRamassage();
            gestionMontage();

            // ── LED ───────────────────────────────────────────
            mettreAJourLED();

            // ── Télémétrie ────────────────────────────────────
            telemetry.addData("Tag", tagVisible ? String.format("%.0fcm  tx=%.1f°", distanceCourante, dernierTx) : "absent");
            telemetry.addData("Turret", String.format("pwr=%.2f  %s", dernierePuissanceTurret,
                    gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1 ? "[OVERRIDE L2/R2]" : "[AUTO]"));
            telemetry.addData("Shooter", shooterActif
                    ? (shooterPret()
                    ? String.format("PRET!  %.0f/%.0f RPM", ticksEnRpm(shooter.getVelocity()), vitesseCourante)
                    : String.format("%.0f/%.0f RPM", ticksEnRpm(shooter.getVelocity()), vitesseCourante))
                    : "OFF");
            telemetry.update();
        }

        arret();
    }

    // ════════════════════════════════════════════════════════
    //  TURRET + DISTANCE + INTERPOLATION
    //
    //  ← MODIF PRINCIPALE : tracking permanent.
    //
    //  Logique de priorité (dans l'ordre) :
    //   1. L2/R2 pressés  → override manuel, turret obéit aux gâchettes
    //                        MAIS Limelight est toujours lu pour la distance
    //   2. Tag frais visible → suivi PD normal
    //   3. Tag vieux (100-300ms) → maintien de la dernière puissance (inertie)
    //   4. Tag absent (>300ms) → freinage progressif puis arrêt
    //
    //  Le shooter et l'angle sont toujours mis à jour si on a une distance
    //  fraîche (même pendant l'override L2/R2).
    // ════════════════════════════════════════════════════════

    private void gestionTurretEtTir() {
        float trigG = gamepad1.left_trigger;
        float trigD = gamepad1.right_trigger;
        boolean overrideManuel = (trigD > 0.1 || trigG > 0.1); // ← MODIF : bool séparé

        // Lecture Limelight — une seule fois par loop
        LLResult result = limelight.getLatestResult();
        boolean resultFrais = (result != null && result.isValid() && result.getStaleness() <= 300);
        FiducialResult tag  = resultFrais ? trouverTag(result) : null;

        if (tag != null) {
            // ── Tag frais ─────────────────────────────────────
            tagVisible = true;
            tempsDepuisTag.reset(); // ← MODIF : reset le timer de perte de tag

            // Caméra inversée — miroir horizontal et vertical
            double txBrut    = tag.getTargetXDegrees();
            double txCorrige = -txBrut - TX_OFFSET_DEG;
            double tyCorrige = -tag.getTargetYDegrees();
            dernierTx        = txCorrige; // pour télémétrie

            // Calcul distance depuis ty
            double angleRad = Math.toRadians(ANGLE_CAMERA + tyCorrige);
            if (Math.abs(angleRad) > 0.01) {
                double d = (H_CAMERA - H_TAG) / Math.tan(angleRad);
                distanceCourante = clamp(d, CALIBRATION[0][0], CALIBRATION[CALIBRATION.length-1][0]);
            }

            // Interpolation → angle servo + vitesse
            double[] tir      = interpoler(distanceCourante);
            angleServoCourant = tir[0];
            vitesseCourante   = tir[1];

            // Angle servo appliqué en permanence (même si shooter OFF)
            servoAngleShooter.setPosition(angleServoCourant);

            // ── Commande turret ────────────────────────────────
            if (overrideManuel) {
                // Override L2/R2 — obéit aux gâchettes
                // ← MODIF : on ne coupe PAS tagVisible, on ne remet PAS dernierTx à 0
                double puissance = (trigD > 0.1) ? trigD * 0.5 : -trigG * 0.5;
                servoTurret.setPower(puissance);
                dernierePuissanceTurret = puissance;
                // Reset le PD pour éviter un sursaut quand on relâche
                tempsTurret.reset();
                // ← MODIF : on ne touche PAS à dernierTx ici (conservé pour reprise auto)
            } else {
                // Suivi PD automatique
                double dt      = tempsTurret.seconds();
                double derivee = (dt > 0.001) ? (txCorrige - dernierTx) / dt : 0;
                tempsTurret.reset();

                double p = 0;
                if (Math.abs(txCorrige) > DEADBAND_TURRET) {
                    p = KP_TURRET * txCorrige + KD_TURRET * derivee;
                    if      (p > 0 && p <  MIN_POWER_TURRET) p =  MIN_POWER_TURRET;
                    else if (p < 0 && p > -MIN_POWER_TURRET) p = -MIN_POWER_TURRET;
                    p = clamp(p, -MAX_POWER_TURRET, MAX_POWER_TURRET);
                }
                servoTurret.setPower(p);
                dernierePuissanceTurret = p;
            }

        } else {
            // ── Tag absent ou vieux ───────────────────────────
            tagVisible = false;
            long msPerdus = (long) tempsDepuisTag.milliseconds();

            if (overrideManuel) {
                // Override L2/R2 même sans tag — commande directe
                double puissance = (trigD > 0.1) ? trigD * 0.5 : -trigG * 0.5;
                servoTurret.setPower(puissance);
                dernierePuissanceTurret = puissance;
            } else if (msPerdus < 100) {
                // ← MODIF : inertie courte — maintien de la dernière puissance
                // Le turret continue sur sa lancée 100ms avant de freiner
                servoTurret.setPower(dernierePuissanceTurret);
            } else if (msPerdus < 400) {
                // ← MODIF : freinage progressif sur 300ms (100ms → 400ms)
                double ratio  = 1.0 - (msPerdus - 100.0) / 300.0;
                double pFrein = dernierePuissanceTurret * ratio;
                servoTurret.setPower(pFrein);
            } else {
                // ← MODIF : tag perdu depuis >400ms → arrêt complet
                servoTurret.setPower(0);
                dernierePuissanceTurret = 0;
                dernierTx = 0;
            }
            // Pas de mise à jour de angle/vitesse si pas de tag frais
        }
    }

    // ════════════════════════════════════════════════════════
    //  INTERPOLATION LINÉAIRE entre les points de CALIBRATION
    // ════════════════════════════════════════════════════════

    private double[] interpoler(double distCm) {
        int n = CALIBRATION.length;
        if (distCm <= CALIBRATION[0][0])
            return new double[]{ CALIBRATION[0][1], CALIBRATION[0][2] };
        if (distCm >= CALIBRATION[n-1][0])
            return new double[]{ CALIBRATION[n-1][1], CALIBRATION[n-1][2] };

        for (int i = 0; i < n - 1; i++) {
            if (distCm >= CALIBRATION[i][0] && distCm <= CALIBRATION[i+1][0]) {
                double t       = (distCm - CALIBRATION[i][0]) / (CALIBRATION[i+1][0] - CALIBRATION[i][0]);
                double angle   = CALIBRATION[i][1] + t * (CALIBRATION[i+1][1] - CALIBRATION[i][1]);
                double vitesse = CALIBRATION[i][2] + t * (CALIBRATION[i+1][2] - CALIBRATION[i][2]);
                return new double[]{ Math.min(angle, 0.306), vitesse };
            }
        }
        return new double[]{ CALIBRATION[0][1], CALIBRATION[0][2] };
    }

    // ════════════════════════════════════════════════════════
    //  SHOOTER
    //
    //  ← MODIF : le shooter s'active automatiquement dès qu'un
    //  tag est visible (à la bonne vitesse interpolée).
    //  Triangle = forcer l'extinction manuelle.
    //  Si le tag disparaît >500ms, le shooter s'éteint seul.
    // ════════════════════════════════════════════════════════

    private void configurerPID() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void gestionShooter() {
        // ← MODIF : activation auto si tag visible
        if (tagVisible && !shooterActif) {
            shooterActif = true;
        }

        // ← MODIF : extinction auto si tag perdu depuis >500ms
        if (!tagVisible && tempsDepuisTag.milliseconds() > SHOOTER_TAG_TIMEOUT_MS) {
            shooterActif = false;
        }

        // Triangle = extinction manuelle forcée (même avec tag visible)
        if (debounceShooter.milliseconds() > 150 && gamepad1.triangle) {
            shooterActif = false;
            debounceShooter.reset();
            // Pour réactiver après Triangle : il faut perdre le tag puis le retrouver,
            // ou appuyer à nouveau sur Triangle quand tagVisible est true.
            // Alternative plus simple : double appui Triangle = réactivation
        }

        shooter.setVelocity(shooterActif ? rpmEnTicks(vitesseCourante) : 0);
    }

    private boolean shooterPret() {
        return shooterActif
                && Math.abs(ticksEnRpm(shooter.getVelocity()) - vitesseCourante) < SHOOTER_TOLERANCE_RPM;
    }

    private void gestionMonterBalles() {
        if (!shooterPret()) return;
        if (debounceMonter.milliseconds() > 200 && gamepad1.circle) {
            servoMoteurRamassageBalle.setPower(-0.6);
            montageGauche.setPower(1.0);
            montageDroit.setPower(1.0);
            debounceMonter.reset();
        }
    }

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT FLUIDE + PINPOINT
    // ════════════════════════════════════════════════════════

    private void deplacementFluide() {
        double forward = appliquerZoneMorte(-gamepad1.right_stick_y);
        double right   = appliquerZoneMorte( gamepad1.right_stick_x);
        double rotate  = appliquerZoneMorte( gamepad1.left_stick_x);

        pinpoint.update();
        double cx = 0, cy = 0;
        if (forward == 0 && right == 0 && rotate == 0) {
            double vx = pinpoint.getVelX(DistanceUnit.MM);
            double vy = pinpoint.getVelY(DistanceUnit.MM);
            if (Math.sqrt(vx*vx + vy*vy) > SEUIL_STAB) {
                cx = clamp(-vx * KP_STAB / 1000.0, -MAX_STAB, MAX_STAB);
                cy = clamp(-vy * KP_STAB / 1000.0, -MAX_STAB, MAX_STAB);
            }
        }

        double fl = forward+cy + right+cx + rotate;
        double fr = forward+cy - right-cx - rotate;
        double br = forward+cy + right+cx - rotate;
        double bl = forward+cy - right-cx + rotate;

        double max = Math.max(Math.max(Math.abs(fl),Math.abs(fr)),Math.max(Math.abs(bl),Math.abs(br)));
        if (max > 1.0) { fl/=max; fr/=max; bl/=max; br/=max; }

        puissanceAvantGaucheActuelle   = appliquerRampe(puissanceAvantGaucheActuelle,   fl);
        puissanceAvantDroitActuelle    = appliquerRampe(puissanceAvantDroitActuelle,    fr);
        puissanceArriereGaucheActuelle = appliquerRampe(puissanceArriereGaucheActuelle, bl);
        puissanceArriereDroitActuelle  = appliquerRampe(puissanceArriereDroitActuelle,  br);

        moteurAvantGauche.setPower(vitesseDeplacement   * puissanceAvantGaucheActuelle);
        moteurAvantDroit.setPower(vitesseDeplacement    * puissanceAvantDroitActuelle);
        moteurArriereGauche.setPower(vitesseDeplacement * puissanceArriereGaucheActuelle);
        moteurArriereDroit.setPower(vitesseDeplacement  * puissanceArriereDroitActuelle);
    }

    private void gestionVitesse() {
        if (debounceVitesse.milliseconds() > 300) {
            if (gamepad1.dpad_up   && !gamepad1.dpad_down) { vitesseDeplacement = Math.min(vitesseDeplacement+0.1,1.0); debounceVitesse.reset(); }
            if (gamepad1.dpad_down && !gamepad1.dpad_up)   { vitesseDeplacement = Math.max(vitesseDeplacement-0.1,0.1); debounceVitesse.reset(); }
        }
        telemetry.addData("Vitesse", String.format("%.0f%%", vitesseDeplacement * 100));
    }

    // ════════════════════════════════════════════════════════
    //  RAMASSAGE
    // ════════════════════════════════════════════════════════

    private void gestionRamassage() {
        if (debounceRamassage.milliseconds() > 200 && gamepad1.left_bumper) {
            estRamassageActif = !estRamassageActif;
            debounceRamassage.reset();
        }
        if (estRamassageActif) {
            intakeMoteur.setPower(1.0);
            servoMoteurRamassageBalle.setPower(-1.0);
            servoRamassageGauche.setPower(-0.3);
            servoRamassageDroit.setPower(0.3);
            montageGauche.setPower(-1.0);
            montageDroit.setPower(-1.0);
        } else {
            intakeMoteur.setPower(0);
            servoMoteurRamassageBalle.setPower(0);
        }
    }

    // ════════════════════════════════════════════════════════
    //  MONTAGE
    // ════════════════════════════════════════════════════════

    private void gestionMontage() {
        if (debounceMontage.milliseconds() > 400 && gamepad1.cross) {
            montageActif = !montageActif;
            debounceMontage.reset();
        }
        if (montageActif) {
            servoRamassageGauche.setPower(1.0);
            servoRamassageDroit.setPower(-1.0);
            servoMoteurRamassageBalle.setPower(-0.5);
            montageGauche.setPower(1.0);
            montageDroit.setPower(1.0);
            intakeMoteur.setPower(1.0);
        } else {
            servoRamassageGauche.setPower(0);
            servoRamassageDroit.setPower(0);
            servoMoteurRamassageBalle.setPower(0);
            montageGauche.setPower(0);
            montageDroit.setPower(0);
        }
    }

    // ════════════════════════════════════════════════════════
    //  LED
    // ════════════════════════════════════════════════════════

    private void mettreAJourLED() {
        if      (estRamassageActif)              light.setPosition(LED_ORANGE);
        else if (shooterActif && shooterPret())  light.setPosition(LED_VERT);
        else if (shooterActif)                   light.setPosition(LED_ROUGE);
        else if (tagVisible)                     light.setPosition(LED_JAUNE);
        else                                     light.setPosition(LED_BLEU);
    }

    // ════════════════════════════════════════════════════════
    //  ARRÊT
    // ════════════════════════════════════════════════════════

    private void arret() {
        moteurAvantGauche.setPower(0);    moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);  moteurArriereDroit.setPower(0);
        shooter.setVelocity(0);
        intakeMoteur.setPower(0);
        montageGauche.setPower(0);        montageDroit.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoRamassageDroit.setPower(0);  servoRamassageGauche.setPower(0);
        servoAngleShooter.setPosition(CALIBRATION[0][1]);
        light.setPosition(LED_OFF);
        limelight.stop();
    }

    // ════════════════════════════════════════════════════════
    //  INITIALISATION
    // ════════════════════════════════════════════════════════

    private void initHardware() {
        moteurAvantDroit    = hardwareMap.get(DcMotor.class,   "avantDroit");
        moteurAvantGauche   = hardwareMap.get(DcMotor.class,   "avantGauche");
        moteurArriereDroit  = hardwareMap.get(DcMotor.class,   "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class,   "dosGauche");
        shooter             = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMoteur        = hardwareMap.get(DcMotor.class,   "intake");
        montageGauche       = hardwareMap.get(DcMotor.class,   "montageG");
        montageDroit        = hardwareMap.get(DcMotor.class,   "montageD");
        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageDroit       = hardwareMap.get(CRServo.class, "ramassageD");
        servoRamassageGauche      = hardwareMap.get(CRServo.class, "ramassageG");
        servoTurret               = hardwareMap.get(CRServo.class, "turret");
        servoAngleShooter         = hardwareMap.get(Servo.class,   "angleShooter");
        light                     = hardwareMap.get(Servo.class,   "light");
        imu      = hardwareMap.get(IMU.class,                    "imu");
        limelight = hardwareMap.get(Limelight3A.class,           "limelight");
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, "odoComputer");

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

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        pinpoint.setOffsets(-17.5, 16.5, DistanceUnit.CM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        servoAngleShooter.setPosition(CALIBRATION[0][1]);
        light.setPosition(LED_OFF);
    }

    // ════════════════════════════════════════════════════════
    //  UTILITAIRES
    // ════════════════════════════════════════════════════════

    private FiducialResult trouverTag(LLResult result) {
        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;
        FiducialResult best = null;
        double minTx = Double.MAX_VALUE;
        for (FiducialResult f : tags) {
            if (Math.abs(f.getTargetXDegrees()) < minTx) {
                minTx = Math.abs(f.getTargetXDegrees());
                best  = f;
            }
        }
        return best;
    }

    private double appliquerRampe(double actuelle, double cible) {
        if (cible == 0 && Math.abs(actuelle) < 0.05) return 0;
        return actuelle + ((cible == 0 ? TAUX_DECELERATION : TAUX_ACCELERATION)) * (cible - actuelle);
    }

    private double appliquerZoneMorte(double v) { return Math.abs(v) < SEUIL_MORT ? 0 : v; }
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }
}
