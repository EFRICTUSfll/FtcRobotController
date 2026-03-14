package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * ============================================================
 *  OUTIL DE CALIBRATION v2 - Servo 300° GoBilda + Shooter
 * ============================================================
 *
 *  ÉTAPE 1 — MODE DÉCOUVERTE SERVO (avant la calibration distance)
 *  ───────────────────────────────────────────────────────────────
 *  Sert à trouver les limites physiques réelles de ton servo 300°.
 *
 *  Contrôles en mode découverte :
 *    JOYSTICK GAUCHE Y  → Bouge le servo doucement (haut/bas)
 *    CROIX              → Sauvegarde position MIN (angle le plus bas)
 *    TRIANGLE           → Sauvegarde position MAX (angle le plus haut)
 *    CERCLE             → Passe en MODE CALIBRATION DISTANCE
 *
 *  À faire : bouge le servo, observe physiquement le shooter,
 *  sauvegarde MIN quand c'est le plus bas, MAX quand c'est le plus haut.
 *  Puis CERCLE pour passer à la calibration.
 *
 *  ÉTAPE 2 — MODE CALIBRATION DISTANCE
 *  ───────────────────────────────────────────────────────────────
 *  Contrôles :
 *    JOYSTICK GAUCHE Y  → Angle shooter (continu, précis)
 *    DPAD UP/DOWN       → RPM ±50
 *    DPAD LEFT/RIGHT    → RPM ±200
 *    R1                 → Active/désactive le shooter
 *    CROIX              → Sauvegarde ce point (distance + angle + RPM)
 *    CERCLE             → Efface le dernier point sauvegardé
 *    TRIANGLE           → Retour mode découverte servo
 *
 *  PROCÉDURE CALIBRATION :
 *    1. Tag ID 20 collé sur le panier à sa hauteur réelle
 *    2. ⚠️ Mesure HAUTEUR_TAG_CM et mets la vraie valeur (ligne ~85)
 *    3. Place robot à 50cm, règle angle+RPM jusqu'à marquer → CROIX
 *    4. Recule de 25cm, recommence jusqu'à ~300cm
 *    5. Note tous les points affichés → envoie-les pour l'étape finale
 * ============================================================
 */
@TeleOp(name = "Calibration Shooter", group = "Calibration")
public class CalibrationShooter extends LinearOpMode {

    // ===================================
    // HARDWARE
    // ===================================
    private Limelight3A limelight;
    private DcMotorEx   shooter;
    private Servo       servoAngleShooter;

    // ===================================
    // LIMELIGHT
    // ===================================
    private static final int    LIMELIGHT_PIPELINE        = 0;
    private static final int    DESIRED_TAG_ID            = 20;
    private static final double HAUTEUR_CAMERA_CM         = 36.0;
    private static final double HAUTEUR_TAG_CM            = 75.0;
    private static final double ANGLE_INCLINAISON_CAM_DEG = 0.0;   // ⚠️ TODO : angle de montage de ta Limelight si inclinée

    // ===================================
    // SHOOTER PID
    // ===================================
    private static final double SHOOTER_P = 15.0;
    private static final double SHOOTER_I = 5.1;
    private static final double SHOOTER_D = 0.5;
    private static final double SHOOTER_F = 10.5;

    // ===================================
    // SERVO 300° GOBILDA
    // ===================================
    // Ces valeurs seront découvertes en ÉTAPE 1 et sauvegardées ici
    // Elles sont initialisées à la plage complète par défaut
    private double servoMin     = 0.0;   // Sera mis à jour en mode découverte
    private double servoMax     = 1.0;   // Sera mis à jour en mode découverte
    private boolean minSauvegarde = false;
    private boolean maxSauvegarde = false;

    // Position actuelle du servo
    private double servoPosition = 0.5;

    // ===================================
    // SHOOTER
    // ===================================
    private double  rpmActuel    = 1500.0;
    private boolean shooterActif = false;

    private static final double RPM_MIN = 300.0;
    private static final double RPM_MAX = 5800.0; // GoBilda RS-555 : 6000 RPM max, marge de sécurité

    // GoBilda 5203 RS-555 1:1
    // 28 PPR en quadrature = 112 ticks/rev (comptage 4x par défaut FTC SDK)
    // 6000 RPM max à 12V
    private static final double TICKS_PAR_REV = 112.0;

    // ===================================
    // MODES
    // ===================================
    private enum Mode { DECOUVERTE_SERVO, CALIBRATION_DISTANCE }
    private Mode modeActuel = Mode.DECOUVERTE_SERVO;

    // ===================================
    // POINTS DE CALIBRATION SAUVEGARDÉS
    // ===================================
    private double[][] points = new double[20][3]; // [distance, angle, rpm]
    private int nbPoints = 0;

    // ===================================
    // TIMERS DEBOUNCE
    // ===================================
    private ElapsedTime debounceMin     = new ElapsedTime();
    private ElapsedTime debounceMax     = new ElapsedTime();
    private ElapsedTime debounceMode    = new ElapsedTime();
    private ElapsedTime debounceShooter = new ElapsedTime();
    private ElapsedTime debounceSave    = new ElapsedTime();
    private ElapsedTime debounceDelete  = new ElapsedTime();
    private ElapsedTime debounceRPM     = new ElapsedTime();

    // ===================================
    // RUNOPMODE
    // ===================================
    @Override
    public void runOpMode() {
        initialisationHardware();
        initialisationLimelight();

        servoAngleShooter.setPosition(servoPosition);

        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("   CALIBRATION SHOOTER v2");
        telemetry.addLine("   Servo GoBilda 300° + Limelight");
        telemetry.addLine("════════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("⚠️  AVANT DE COMMENCER :");
        telemetry.addLine("   Mesure la hauteur du CENTRE du");
        telemetry.addLine("   tag ID 20 sur le panier (en cm)");
        telemetry.addLine("   et mets la valeur dans le code");
        telemetry.addLine("   à la ligne HAUTEUR_TAG_CM (~85)");
        telemetry.addLine("");
        telemetry.addData(">", "Appuie START pour commencer");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            switch (modeActuel) {
                case DECOUVERTE_SERVO:
                    boucleDecouverteServo();
                    break;
                case CALIBRATION_DISTANCE:
                    boucleCalibrationDistance();
                    break;
            }

            telemetry.update();
            sleep(20);
        }

        shooter.setVelocity(0);
        limelight.stop();
    }

    // ===================================
    // ÉTAPE 1 : DÉCOUVERTE SERVO
    // ===================================
    private void boucleDecouverteServo() {

        // Joystick gauche Y → bouge le servo doucement
        double joystick = -gamepad1.left_stick_y; // -1 à +1
        // Zone morte
        if (Math.abs(joystick) > 0.05) {
            servoPosition += joystick * 0.003; // Incrément très doux
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
            servoAngleShooter.setPosition(servoPosition);
        }

        // CROIX → sauvegarde MIN
        if (debounceMin.milliseconds() > 400 && gamepad1.cross) {
            servoMin = servoPosition;
            minSauvegarde = true;
            debounceMin.reset();
        }

        // TRIANGLE → sauvegarde MAX
        if (debounceMax.milliseconds() > 400 && gamepad1.triangle) {
            servoMax = servoPosition;
            maxSauvegarde = true;
            debounceMax.reset();
        }

        // CERCLE → passe en mode calibration (seulement si min ET max sauvegardés)
        if (debounceMode.milliseconds() > 500 && gamepad1.circle) {
            if (minSauvegarde && maxSauvegarde) {
                // Assure que min < max (inverse si nécessaire)
                if (servoMin > servoMax) {
                    double temp = servoMin;
                    servoMin = servoMax;
                    servoMax = temp;
                }
                // Position de départ = milieu de la plage découverte
                servoPosition = (servoMin + servoMax) / 2.0;
                servoAngleShooter.setPosition(servoPosition);
                modeActuel = Mode.CALIBRATION_DISTANCE;
                debounceMode.reset();
            }
        }

        // ── Affichage ──
        telemetry.addLine("════════ MODE DÉCOUVERTE SERVO ════════");
        telemetry.addLine("");
        telemetry.addLine("  JOYSTICK GAUCHE Y → Bouge le servo");
        telemetry.addLine("  CROIX             → Sauvegarde MIN");
        telemetry.addLine("  TRIANGLE          → Sauvegarde MAX");
        telemetry.addLine("  CERCLE            → Passer calibration");
        telemetry.addLine("                      (MIN + MAX requis)");
        telemetry.addLine("");
        telemetry.addData("📍 Position actuelle", "%.4f", servoPosition);
        telemetry.addLine("");

        if (minSauvegarde) {
            telemetry.addData("✅ MIN sauvegardé", "%.4f  (angle le plus bas)", servoMin);
        } else {
            telemetry.addData("⬜ MIN", "Non sauvegardé — bouge jusqu'au plus bas → CROIX");
        }

        if (maxSauvegarde) {
            telemetry.addData("✅ MAX sauvegardé", "%.4f  (angle le plus haut)", servoMax);
        } else {
            telemetry.addData("⬜ MAX", "Non sauvegardé — bouge jusqu'au plus haut → TRIANGLE");
        }

        telemetry.addLine("");
        if (minSauvegarde && maxSauvegarde) {
            double plage = Math.abs(servoMax - servoMin);
            telemetry.addData("📐 Plage utilisable", "%.4f  (sur 1.0 total)", plage);
            telemetry.addLine("  ✅ Prêt ! Appuie CERCLE pour calibrer.");
        } else {
            telemetry.addLine("  ⚠️  Sauvegarde MIN et MAX pour continuer.");
        }
    }

    // ===================================
    // ÉTAPE 2 : CALIBRATION DISTANCE
    // ===================================
    private void boucleCalibrationDistance() {

        double distanceCM = calculerDistance();

        // ── Angle servo via joystick gauche Y ──
        double joystick = -gamepad1.left_stick_y;
        if (Math.abs(joystick) > 0.05) {
            // Bouge dans la plage MIN→MAX découverte
            servoPosition += joystick * 0.003;
            servoPosition = Math.max(servoMin, Math.min(servoMax, servoPosition));
            servoAngleShooter.setPosition(servoPosition);
        }

        // ── RPM via D-PAD ──
        if (debounceRPM.milliseconds() > 100) {
            if (gamepad1.dpad_up)    { rpmActuel += 50;  debounceRPM.reset(); }
            if (gamepad1.dpad_down)  { rpmActuel -= 50;  debounceRPM.reset(); }
            if (gamepad1.dpad_right) { rpmActuel += 200; debounceRPM.reset(); }
            if (gamepad1.dpad_left)  { rpmActuel -= 200; debounceRPM.reset(); }
        }
        rpmActuel = Math.max(RPM_MIN, Math.min(RPM_MAX, rpmActuel));

        // ── Shooter ON/OFF via R1 ──
        if (debounceShooter.milliseconds() > 200 && gamepad1.right_bumper) {
            shooterActif = !shooterActif;
            debounceShooter.reset();
        }

        if (shooterActif) {
            double ticksParSec = (rpmActuel / 60.0) * TICKS_PAR_REV;
            shooter.setVelocity(ticksParSec);
        } else {
            shooter.setVelocity(0);
        }

        // ── Sauvegarder un point (CROIX) ──
        if (debounceSave.milliseconds() > 500 && gamepad1.cross) {
            if (distanceCM > 0 && nbPoints < 20) {
                points[nbPoints][0] = Math.round(distanceCM * 10.0) / 10.0;
                points[nbPoints][1] = Math.round(servoPosition * 10000.0) / 10000.0;
                points[nbPoints][2] = Math.round(rpmActuel);
                nbPoints++;
                debounceSave.reset();
            }
        }

        // ── Effacer dernier point (CERCLE) ──
        if (debounceDelete.milliseconds() > 500 && gamepad1.circle) {
            if (nbPoints > 0) { nbPoints--; }
            debounceDelete.reset();
        }

        // ── Retour découverte servo (TRIANGLE) ──
        if (debounceMode.milliseconds() > 500 && gamepad1.triangle) {
            shooter.setVelocity(0);
            shooterActif = false;
            modeActuel = Mode.DECOUVERTE_SERVO;
            debounceMode.reset();
        }

        // ── Affichage ──
        telemetry.addLine("════════ MODE CALIBRATION DISTANCE ════════");
        telemetry.addLine("");
        telemetry.addLine("  JOYSTICK G Y  → Angle shooter");
        telemetry.addLine("  DPAD UP/DOWN  → RPM ±50");
        telemetry.addLine("  DPAD L/R      → RPM ±200");
        telemetry.addLine("  R1            → Shooter ON/OFF");
        telemetry.addLine("  CROIX         → Sauvegarder point");
        telemetry.addLine("  CERCLE        → Effacer dernier");
        telemetry.addLine("  TRIANGLE      → Retour découverte servo");
        telemetry.addLine("");

        // Distance
        telemetry.addLine("──── LIMELIGHT ────");
        if (distanceCM > 0) {
            telemetry.addData("📏 Distance tag 20", "%.1f cm", distanceCM);
        } else {
            telemetry.addData("📏 Distance", "❌ Tag ID 20 non détecté");
        }

        // Données brutes limelight
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f.getFiducialId() == DESIRED_TAG_ID) {
                        telemetry.addData("   TX", "%.2f°", f.getTargetXDegrees());
                        telemetry.addData("   TY", "%.2f°", f.getTargetYDegrees());
                    }
                }
            }
        }

        // Shooter
        telemetry.addLine("──── SHOOTER ────");
        telemetry.addData("🎯 Angle servo",  "%.4f  (plage %.4f → %.4f)", servoPosition, servoMin, servoMax);
        telemetry.addData("⚡ RPM cible",    "%.0f", rpmActuel);
        if (shooterActif) {
            double rpmReel = (shooter.getVelocity() / TICKS_PAR_REV) * 60.0;
            telemetry.addData("🔴 Shooter",     "ACTIF");
            telemetry.addData("   RPM réel",    "%.0f  (erreur: %.0f)", rpmReel, rpmActuel - rpmReel);
        } else {
            telemetry.addData("⚫ Shooter",     "ARRÊTÉ  (R1 pour activer)");
        }

        // Points sauvegardés
        telemetry.addLine("");
        telemetry.addLine("──── POINTS SAUVEGARDÉS ────");
        if (nbPoints == 0) {
            telemetry.addLine("  Aucun point. Règle jusqu'à marquer → CROIX.");
        } else {
            telemetry.addLine("  #   Dist(cm)  Angle    RPM");
            telemetry.addLine("  ─────────────────────────────");
            for (int i = 0; i < nbPoints; i++) {
                telemetry.addData(
                        String.format("  %2d", i + 1),
                        "%.0f cm   %.4f   %.0f RPM",
                        points[i][0], points[i][1], points[i][2]
                );
            }
            telemetry.addLine("");
            telemetry.addLine("  ✅ Copie ces valeurs et envoie-les moi !");
        }
    }

    // ===================================
    // CALCUL DISTANCE LIMELIGHT
    // ===================================
    private double calculerDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return -1;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == DESIRED_TAG_ID) {
                double ty = fiducial.getTargetYDegrees();
                double angleTotalRad = Math.toRadians(ANGLE_INCLINAISON_CAM_DEG + ty);

                if (Math.abs(Math.tan(angleTotalRad)) < 0.001) return -1;

                double distance = (HAUTEUR_TAG_CM - HAUTEUR_CAMERA_CM) / Math.tan(angleTotalRad);
                return (distance > 0 && distance < 600) ? distance : -1;
            }
        }
        return -1;
    }

    // ===================================
    // INIT LIMELIGHT
    // ===================================
    private void initialisationLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        sleep(500);
    }

    // ===================================
    // INIT HARDWARE
    // ===================================
    private void initialisationHardware() {
        shooter           = hardwareMap.get(DcMotorEx.class, "shooter");
        servoAngleShooter = hardwareMap.get(Servo.class,     "angleShooter");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}