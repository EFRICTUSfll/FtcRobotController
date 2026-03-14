package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ════════════════════════════════════════════════════════════
 *  SHOOTER TUNER — Calibration RPM / Ticks / Speed
 *
 *  Naviguer avec la manette :
 *
 *  DPAD_UP    → vitesse +1 pas
 *  DPAD_DOWN  → vitesse -1 pas
 *  DPAD_RIGHT → passer au preset suivant
 *  DPAD_LEFT  → passer au preset précédent
 *
 *  Y (Triangle) → grand pas (×10)
 *  A (Croix)    → petit pas (÷10)
 *  X (Carré)    → toggle shooter ON/OFF
 *  B (Rond)     → reset à 0
 *  RB (R1)      → aller au MAX direct
 *
 *  MODE D'AFFICHAGE (bumpers) :
 *  LB (L1)      → changer mode : RPM / TICKS / SPEED%
 *
 * ════════════════════════════════════════════════════════════
 *
 *  GoBilda Yellow Jacket 6000 RPM
 *  28 ticks/tour (encoder intégré, pas de réduction)
 *  → Max ticks/s = 6000 / 60 * 28 = 2800 ticks/s
 *
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "MAX - FEST", group = "FIN")
public class MAXFirst extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────
    DcMotorEx shooter;

    // ── Moteur : GoBilda Yellow Jacket 6000 RPM ───────────
    //    28 ticks / révolution (motor shaft, pas de réduction)
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM       = 6000.0;
    private static final double MAX_TICKS_S   = MAX_RPM / 60.0 * TICKS_PER_REV; // 2800.0
    // ── PIDF (identique à MaxF) ───────────────────────────
    private static final double SHOOTER_P = 15.0;
    private static final double SHOOTER_I =  5.1;
    private static final double SHOOTER_D =  0.5;
    private static final double SHOOTER_F = 10.5;

    // ── Modes d'affichage / navigation ────────────────────
    private static final int MODE_RPM    = 0;
    private static final int MODE_TICKS  = 1;
    private static final int MODE_SPEED  = 2;
    private int mode = MODE_RPM;

    // ── Taille des pas ────────────────────────────────────
    //    Normal  : ~1% du max
    //    Grand   : ~10% du max (Y enfoncé)
    //    Petit   : ~0.1% du max (A enfoncé)
    private static final double PAS_NORMAL_RPM   =  60.0;   //  1%  de 6000
    private static final double PAS_GRAND_RPM    = 600.0;   // 10%  de 6000
    private static final double PAS_PETIT_RPM    =   6.0;   //  0.1% de 6000

    private static final double PAS_NORMAL_TICKS =  28.0;   // 1 révolution/s
    private static final double PAS_GRAND_TICKS  = 280.0;
    private static final double PAS_PETIT_TICKS  =   1.0;

    private static final double PAS_NORMAL_SPEED =  1.0;    // 1%
    private static final double PAS_GRAND_SPEED  = 10.0;
    private static final double PAS_PETIT_SPEED  =  0.1;

    // ── Presets mémorisés (20 paliers de 0 à MAX) ─────────
    //    Calculés automatiquement, modifiables en live
    private final double[] presetsRPM = new double[20];
    private int presetIndex = 0;

    // ── État ──────────────────────────────────────────────
    private double currentRPM    = 0;
    private boolean shooterActif = false;

    // ── Debounce ──────────────────────────────────────────
    private ElapsedTime timerDpad   = new ElapsedTime();
    private ElapsedTime timerMode   = new ElapsedTime();
    private ElapsedTime timerToggle = new ElapsedTime();
    private ElapsedTime timerPreset = new ElapsedTime();

    // ── Debounce delays ──────────────────────────────────
    private static final long DEBOUNCE_DPAD   = 150; // ms entre chaque pas
    private static final long DEBOUNCE_BTN    = 250;

    // ─────────────────────────────────────────────────────
    @Override
    public void runOpMode() {

        // Init hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Générer les 20 presets (0% → 100%)
        for (int i = 0; i < 20; i++) {
            presetsRPM[i] = Math.round(MAX_RPM / 19.0 * i);
        }

        telemetry.addLine("=== SHOOTER TUNER PRÊT ===");
        telemetry.addLine("Appuie sur START");
        telemetry.update();
        waitForStart();

        timerDpad.reset();
        timerMode.reset();
        timerToggle.reset();
        timerPreset.reset();

        while (opModeIsActive()) {

            // ── Modificateur de pas ───────────────────────
            boolean grandPas  = gamepad1.y;
            boolean petitPas  = gamepad1.a;

            // ── Toggle shooter : X ────────────────────────
            if (timerToggle.milliseconds() > DEBOUNCE_BTN && gamepad1.x) {
                shooterActif = !shooterActif;
                timerToggle.reset();
            }

            // ── Reset à 0 : B ─────────────────────────────
            if (gamepad1.b) {
                currentRPM   = 0;
                shooterActif = false;
            }

            // ── Max direct : RB ───────────────────────────
            if (gamepad1.right_bumper) {
                currentRPM   = MAX_RPM;
                shooterActif = true;
            }

            // ── Changer mode d'affichage : LB ─────────────
            if (timerMode.milliseconds() > DEBOUNCE_BTN && gamepad1.left_bumper) {
                mode = (mode + 1) % 3;
                timerMode.reset();
            }

            // ── Navigation vitesse : DPAD ↑ / ↓ ──────────
            if (timerDpad.milliseconds() > DEBOUNCE_DPAD) {
                if (gamepad1.dpad_up) {
                    currentRPM = clamp(currentRPM + getPas(grandPas, petitPas), 0, MAX_RPM);
                    timerDpad.reset();
                }
                if (gamepad1.dpad_down) {
                    currentRPM = clamp(currentRPM - getPas(grandPas, petitPas), 0, MAX_RPM);
                    timerDpad.reset();
                }
            }

            // ── Navigation presets : DPAD ← / → ─────────
            if (timerPreset.milliseconds() > DEBOUNCE_BTN) {
                if (gamepad1.dpad_right) {
                    presetIndex = Math.min(presetIndex + 1, presetsRPM.length - 1);
                    currentRPM  = presetsRPM[presetIndex];
                    timerPreset.reset();
                }
                if (gamepad1.dpad_left) {
                    presetIndex = Math.max(presetIndex - 1, 0);
                    currentRPM  = presetsRPM[presetIndex];
                    timerPreset.reset();
                }
            }

            // ── Appliquer la vitesse au moteur ────────────
            if (shooterActif) {
                double ticksCible = rpmToTicks(currentRPM);
                shooter.setVelocity(ticksCible);
            } else {
                shooter.setVelocity(0);
            }

            // ── Lecture mesures réelles ───────────────────
            double ticksMesures = shooter.getVelocity();
            double rpmMesure    = ticksToRpm(ticksMesures);
            double erreur       = Math.abs(ticksMesures - rpmToTicks(currentRPM));
            boolean pret        = shooterActif && erreur < 30;

            // ══════════════════════════════════════════════
            //  TELEMETRY
            // ══════════════════════════════════════════════
            telemetry.addLine("══════════════════════════════════");
            telemetry.addLine("  SHOOTER TUNER");
            telemetry.addLine("══════════════════════════════════");

            // État shooter
            telemetry.addData("Shooter",     shooterActif ? "▶ ON" : "■ OFF");
            telemetry.addData("Statut",      pret ? "✓ PRÊT" : (shooterActif ? "... en montée" : "—"));

            telemetry.addLine("──────────────────────────────────");

            // Valeurs cibles
            telemetry.addData("Mode",       getModeLabel());
            switch (mode) {
                case MODE_RPM:
                    telemetry.addData("Cible  RPM",    String.format("%.0f / %.0f RPM", currentRPM, MAX_RPM));
                    telemetry.addData("Cible  Ticks",  String.format("%.0f t/s", rpmToTicks(currentRPM)));
                    telemetry.addData("Cible  Speed",  String.format("%.1f %%", currentRPM / MAX_RPM * 100));
                    break;
                case MODE_TICKS:
                    telemetry.addData("Cible  Ticks",  String.format("%.0f / %.0f t/s", rpmToTicks(currentRPM), MAX_TICKS_S));
                    telemetry.addData("Cible  RPM",    String.format("%.0f RPM", currentRPM));
                    telemetry.addData("Cible  Speed",  String.format("%.1f %%", currentRPM / MAX_RPM * 100));
                    break;
                case MODE_SPEED:
                    telemetry.addData("Cible  Speed",  String.format("%.1f %% / 100%%", currentRPM / MAX_RPM * 100));
                    telemetry.addData("Cible  RPM",    String.format("%.0f RPM", currentRPM));
                    telemetry.addData("Cible  Ticks",  String.format("%.0f t/s", rpmToTicks(currentRPM)));
                    break;
            }

            telemetry.addLine("──────────────────────────────────");

            // Valeurs mesurées
            telemetry.addData("Mesure RPM",   String.format("%.0f RPM",  rpmMesure));
            telemetry.addData("Mesure Ticks", String.format("%.0f t/s",  ticksMesures));
            telemetry.addData("Erreur",       String.format("%.0f t/s",  erreur));

            // Barre de progression ASCII
            telemetry.addData("Barre", barreProgression(currentRPM / MAX_RPM));

            telemetry.addLine("──────────────────────────────────");

            // Preset actif
            telemetry.addData("Preset",  String.format("%02d/20 → %.0f RPM", presetIndex + 1, presetsRPM[presetIndex]));

            // Pas actif
            telemetry.addData("Pas",     String.format("%.0f %s", getPas(grandPas, petitPas), getModeUnit()));

            telemetry.addLine("──────────────────────────────────");
            telemetry.addLine("↑↓ vitesse  ←→ preset");
            telemetry.addLine("Y=grand pas  A=petit pas");
            telemetry.addLine("X=ON/OFF  B=reset  RB=MAX");
            telemetry.addLine("LB=changer mode");
            telemetry.addLine("══════════════════════════════════");

            telemetry.update();
        }

        // Arrêt propre
        shooter.setVelocity(0);
    }

    // ─────────────────────────────────────────────────────
    //  UTILITAIRES
    // ─────────────────────────────────────────────────────

    /** Retourne le pas selon le modificateur et le mode actuel */
    private double getPas(boolean grand, boolean petit) {
        switch (mode) {
            case MODE_RPM:
                return grand ? PAS_GRAND_RPM   : petit ? PAS_PETIT_RPM   : PAS_NORMAL_RPM;
            case MODE_TICKS:
                // Convertit le pas ticks → RPM pour navigation cohérente
                double pasT = grand ? PAS_GRAND_TICKS : petit ? PAS_PETIT_TICKS : PAS_NORMAL_TICKS;
                return ticksToRpm(pasT); // on stocke toujours en RPM
            case MODE_SPEED:
                double pasS = grand ? PAS_GRAND_SPEED : petit ? PAS_PETIT_SPEED : PAS_NORMAL_SPEED;
                return pasS / 100.0 * MAX_RPM;
            default:
                return PAS_NORMAL_RPM;
        }
    }

    private double rpmToTicks(double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }

    private double ticksToRpm(double ticks) {
        return ticks * 60.0 / TICKS_PER_REV;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private String getModeLabel() {
        switch (mode) {
            case MODE_RPM:   return "RPM";
            case MODE_TICKS: return "TICKS/S";
            case MODE_SPEED: return "SPEED %";
            default:         return "?";
        }
    }

    private String getModeUnit() {
        switch (mode) {
            case MODE_RPM:   return "RPM";
            case MODE_TICKS: return "t/s";
            case MODE_SPEED: return "%";
            default:         return "";
        }
    }

    /** Barre ASCII de progression ex: [████████░░░░░░░░] 52% */
    private String barreProgression(double ratio) {
        int total  = 16;
        int filled = (int) Math.round(ratio * total);
        filled = Math.max(0, Math.min(total, filled));
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < total; i++) sb.append(i < filled ? "█" : "░");
        sb.append(String.format("] %.0f%%", ratio * 100));
        return sb.toString();
    }
}