package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ════════════════════════════════════════════════════════════
 *  BRICKMAKERS — Team 31563
 *  TEST LIMELIGHT — Turret + AprilTag uniquement
 *
 *  Ce code ne fait QUE :
 *    1. Lire le TX de la Limelight
 *    2. Tourner le turret pour centrer le tag
 *    3. Afficher tout sur la Driver Station
 *
 *  Rien d'autre — pas de shooter, pas de drivetrain, pas de ramassage.
 *  Le but est de vérifier que le turret suit bien le tag.
 *
 *  ════════════════════════════════════════════════════════════
 *  COMMANDES :
 *    L2          → turret gauche (manuel)
 *    R2          → turret droite (manuel)
 *    (rien)      → suivi automatique
 *    dpad_up     → augmente KP de 0.002 (EN DIRECT sans recompiler)
 *    dpad_down   → diminue  KP de 0.002
 *    dpad_right  → augmente DEADBAND de 0.5°
 *    dpad_left   → diminue  DEADBAND de 0.5°
 *    triangle    → inverse le sens du turret (si part dans le mauvais sens)
 *  ════════════════════════════════════════════════════════════
 *
 *  HARDWARE MAP REQUIS :
 *    "limelight"  → Limelight3A
 *    "turret"     → CRServo
 *    "light"      → Servo (LED)  [optionnel, commente si absent]
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "TEST Limelight Turret", group = "Tests")
public class TestLimeLight extends LinearOpMode {

    // ── Hardware ───────────────────────────────────────────
    private Limelight3A limelight;
    private CRServo     turret;
    private Servo       light;

    // ── Paramètres PD — modifiables EN DIRECT avec dpad ───
    private double kp          = 0.025; // gain proportionnel
    private double deadband    = 2.5;   // zone morte en degrés
    private double minPower    = 0.07;  // puissance minimale anti-frottement
    private double maxPower    = 0.60;  // puissance maximale
    private double kdTurret    = 0.003; // gain dérivé
    private boolean inverseSens = false; // triangle pour inverser

    // ── État PD ────────────────────────────────────────────
    private double      dernierTx   = 0.0;
    private ElapsedTime tempsPD     = new ElapsedTime();
    private ElapsedTime debounceKP  = new ElapsedTime();
    private ElapsedTime debounceDB  = new ElapsedTime();
    private ElapsedTime debounceSens= new ElapsedTime();

    // ── LED ────────────────────────────────────────────────
    private static final double LED_BLEU   = 0.611; // init
    private static final double LED_VERT   = 0.500; // centré (tx dans deadband)
    private static final double LED_ROUGE  = 0.160; // tag visible, pas centré
    private static final double LED_OFF    = 0.388; // pas de tag

    @Override
    public void runOpMode() {

        // ── Init ──────────────────────────────────────────
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret    = hardwareMap.get(CRServo.class,     "turret");
        light     = hardwareMap.get(Servo.class,       "light");

        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setPower(0);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        light.setPosition(LED_BLEU);

        telemetry.addLine("════ TEST LIMELIGHT TURRET ════");
        telemetry.addLine("Prêt. Appuie sur START.");
        telemetry.addLine("dpad haut/bas  = ajuste KP");
        telemetry.addLine("dpad g/d       = ajuste DEADBAND");
        telemetry.addLine("Triangle       = inverse sens turret");
        telemetry.addLine("L2/R2          = contrôle manuel");
        telemetry.update();

        waitForStart();
        tempsPD.reset();

        while (opModeIsActive()) {

            // ── Ajustement KP en direct ───────────────────
            if (debounceKP.milliseconds() > 200) {
                if (gamepad1.dpad_up) {
                    kp = Math.min(kp + 0.002, 0.10);
                    debounceKP.reset();
                }
                if (gamepad1.dpad_down) {
                    kp = Math.max(kp - 0.002, 0.002);
                    debounceKP.reset();
                }
            }

            // ── Ajustement DEADBAND en direct ─────────────
            if (debounceDB.milliseconds() > 200) {
                if (gamepad1.dpad_right) {
                    deadband = Math.min(deadband + 0.5, 10.0);
                    debounceDB.reset();
                }
                if (gamepad1.dpad_left) {
                    deadband = Math.max(deadband - 0.5, 0.5);
                    debounceDB.reset();
                }
            }

            // ── Inversion sens ────────────────────────────
            if (debounceSens.milliseconds() > 300 && gamepad1.triangle) {
                inverseSens = !inverseSens;
                debounceSens.reset();
            }

            // ── Lecture Limelight ─────────────────────────
            LLResult result   = limelight.getLatestResult();
            boolean  valide   = (result != null && result.isValid() && result.getStaleness() <= 300);
            FiducialResult tag = valide ? trouverTagLePlusCentre(result) : null;

            // ── Contrôle turret ───────────────────────────
            float  gaucheManuel = gamepad1.left_trigger;
            float  droiteManuel = gamepad1.right_trigger;
            double puissance    = 0;
            String modeStr      = "AUTO";
            String centreStr    = "-";

            if (droiteManuel > 0.1 || gaucheManuel > 0.1) {
                // ── Mode manuel ───────────────────────────
                modeStr = "MANUEL";
                if (droiteManuel > 0.1) puissance =  droiteManuel * 0.5;
                if (gaucheManuel > 0.1) puissance = -gaucheManuel * 0.5;
                dernierTx = 0;
                light.setPosition(LED_OFF);

            } else if (tag != null) {
                // ── Mode auto : suivi PD ──────────────────
                double tx = tag.getTargetXDegrees();

                // Applique l'inversion si triangle pressé
                if (inverseSens) tx = -tx;

                double dt      = tempsPD.seconds();
                double derivee = (dt > 0.001) ? (tx - dernierTx) / dt : 0;
                dernierTx = tx;
                tempsPD.reset();

                if (Math.abs(tx) <= deadband) {
                    puissance = 0;
                    centreStr = "CENTRÉ ✅";
                    light.setPosition(LED_VERT);
                } else {
                    puissance = kp * tx + kdTurret * derivee;
                    if (puissance > 0 && puissance <  minPower) puissance =  minPower;
                    if (puissance < 0 && puissance > -minPower) puissance = -minPower;
                    puissance = Math.max(-maxPower, Math.min(maxPower, puissance));
                    centreStr = String.format("écart %.1f°", tx);
                    light.setPosition(LED_ROUGE);
                }

            } else {
                // ── Pas de tag ────────────────────────────
                modeStr = "PAS DE TAG";
                dernierTx = 0;
                tempsPD.reset();
                light.setPosition(LED_OFF);
            }

            turret.setPower(puissance);

            // ── Télémétrie complète ───────────────────────
            telemetry.addLine("════ TEST LIMELIGHT TURRET ════");
            telemetry.addData("Mode",           modeStr);
            telemetry.addData("Tag visible",    tag != null ? "OUI — ID " + tag.getFiducialId() : "NON");

            if (tag != null) {
                double txRaw = tag.getTargetXDegrees();
                double tyRaw = tag.getTargetYDegrees();
                telemetry.addLine("─── Données brutes Limelight ───");
                telemetry.addData("TX brut (horiz)", String.format("%.2f°", txRaw));
                telemetry.addData("TY brut (vert)",  String.format("%.2f°", tyRaw));
                telemetry.addData("TX utilisé",      String.format("%.2f°  %s",
                        inverseSens ? -txRaw : txRaw,
                        inverseSens ? "(inversé)" : "(normal)"));
                telemetry.addLine("─── Turret ─────────────────────");
                telemetry.addData("Centre",          centreStr);
                telemetry.addData("Puissance",       String.format("%.3f", puissance));
                telemetry.addData("Staleness",       result.getStaleness() + " ms");
            }

            telemetry.addLine("─── Paramètres (ajustables) ────");
            telemetry.addData("KP",              String.format("%.3f  (dpad haut/bas)", kp));
            telemetry.addData("Deadband",        String.format("%.1f°  (dpad g/d)", deadband));
            telemetry.addData("Sens turret",     inverseSens ? "INVERSÉ (triangle=normal)" : "NORMAL (triangle=inverse)");
            telemetry.addLine("────────────────────────────────");
            telemetry.addLine("💡 Note les valeurs KP et Deadband");
            telemetry.addLine("   qui marchent → recopie dans BotFest");
            telemetry.update();
        }

        turret.setPower(0);
        light.setPosition(LED_OFF);
        limelight.stop();
    }

    /**
     * Retourne le tag le plus centré horizontalement (|tx| minimal).
     */
    private FiducialResult trouverTagLePlusCentre(LLResult result) {
        if (result.getFiducialResults() == null
                || result.getFiducialResults().isEmpty()) return null;

        FiducialResult meilleur  = null;
        double         minEcart  = Double.MAX_VALUE;

        for (FiducialResult f : result.getFiducialResults()) {
            double ecart = Math.abs(f.getTargetXDegrees());
            if (ecart < minEcart) {
                minEcart  = ecart;
                meilleur  = f;
            }
        }
        return meilleur;
    }
}