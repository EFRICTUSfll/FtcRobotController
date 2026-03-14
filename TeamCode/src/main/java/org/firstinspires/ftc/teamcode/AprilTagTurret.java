package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * =====================================================
 *  BRICKMAKERS - Team 31563
 * =====================================================
 */
@TeleOp(name = "Brickmakers - AprilTag Turret", group = "Brickmakers")
public class AprilTagTurret extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════
    //  COULEURS LED GoBILDA RGB Indicator Light
    // ═══════════════════════════════════════════════════════════════
    private static final double LED_OFF    = 0.388; // Éteint
    private static final double LED_VERT   = 0.500; // 🟢 Tracking
    private static final double LED_ROUGE  = 0.160; // 🔴 Attente
    private static final double LED_ORANGE = 0.220; // 🟠 Scan
    private static final double LED_BLEU   = 0.750; // 🔵 Démarrage

    // ═══════════════════════════════════════════════════════════════
    //  CONSTANTES TURRET (CRServo)
    // ═══════════════════════════════════════════════════════════════

    // CRServo : puissance entre -1.0 et +1.0
    private static final double CRSERVO_STOP       =  0.0;  // Arrêt
    private static final double CRSERVO_SCAN_POWER =  0.3;  // Vitesse scan (ajustable)
    private static final double CRSERVO_MAX_POWER  =  0.6;  // Puissance max du suivi P

    // Zone morte : si le tag est dans ce range, on n'applique pas de correction
    private static final double DEADBAND_DEGREES   =  2.5;

    // Gain proportionnel (P) — augmenter si lent, diminuer si oscillation
    private static final double KP = 0.025;

    // ═══════════════════════════════════════════════════════════════
    //  CONSTANTES LIMELIGHT
    // ═══════════════════════════════════════════════════════════════

    // ID de l'AprilTag cible (-1 = n'importe lequel)
    private static final int TARGET_TAG_ID = -1; // (Bleu = 20 | Rouge = 24)

    // Pipeline Limelight pour AprilTag (0 par défaut)
    private static final int LIMELIGHT_PIPELINE = 0;

    // Délai avant reprise du scan après perte du tag
    private static final long LOST_TAG_TIMEOUT_MS = 5000; // 5 secondes

    // ══════════
    //  HARDWARE
    // ══════════
    private Limelight3A limelight;
    private CRServo     turretServo; // ← CRServo (rotation continue)
    private Servo       light;       // ← Servo  (LED RGB)

    // ═══════════════════════════════════════════════════════════════
    //  VARIABLES D'ÉTAT
    // ═══════════════════════════════════════════════════════════════
    private enum TurretState {
        SCANNING,       // Aucun tag : rotation de scan
        TRACKING,       // Tag détecté : suivi proportionnel
        LOST_WAITING    // Tag perdu : attente 3s avant scan
    }

    private TurretState state      = TurretState.SCANNING;
    private ElapsedTime lostTimer  = new ElapsedTime();
    private boolean     tagVisible = false;
    private double      tagTx      = 0.0;

    // Clignotement LED
    private ElapsedTime blinkTimer            = new ElapsedTime();
    private boolean     blinkState            = false;
    private static final long BLINK_PERIOD_MS = 350;

    // ═══════════════════════════════════════════════════════════════
    //  OPMODE
    // ═══════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() {

        // ── Init Hardware ────────────────────────────────────────
        limelight   = hardwareMap.get(Limelight3A.class, "limelight");
        turretServo = hardwareMap.get(CRServo.class,     "turret");   // ← CRServo
        light       = hardwareMap.get(Servo.class,       "light");

        // Direction du CRServo (inverser si le sens de scan est inversé)
        turretServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // ── Config Limelight ─────────────────────────────────────
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        // ── Positions initiales ──────────────────────────────────
        turretServo.setPower(CRSERVO_STOP);
        light.setPosition(LED_BLEU);   // Bleu pendant l'init

        telemetry.addLine("✅ Brickmakers 31563 - Prêt !");
        telemetry.addLine("Appuyez sur START");
        telemetry.update();

        waitForStart();

        blinkTimer.reset();
        lostTimer.reset();

        // ════════════════════════════════════════════════════════
        //  BOUCLE PRINCIPALE
        // ════════════════════════════════════════════════════════
        while (opModeIsActive()) {

            // 1) Lire les données Limelight
            updateLimelightData();

            // 2) Machine à états
            switch (state) {

                // ─── SCANNING ─────────────────────────────────────
                // Aucun tag : on tourne et on clignote en orange
                case SCANNING:
                    blinkLed(LED_ORANGE);

                    if (tagVisible) {
                        // Tag trouvé → passer en suivi
                        state = TurretState.TRACKING;
                        light.setPosition(LED_VERT);
                    } else {
                        turretServo.setPower(CRSERVO_SCAN_POWER); // ← setPower
                    }
                    break;

                // ─── TRACKING ─────────────────────────────────────
                // Tag visible : suivi proportionnel, LED verte fixe
                case TRACKING:
                    light.setPosition(LED_VERT);

                    if (!tagVisible) {
                        // Tag perdu → démarrer le timer de 3 secondes
                        state = TurretState.LOST_WAITING;
                        lostTimer.reset();
                        blinkTimer.reset();
                        turretServo.setPower(CRSERVO_STOP);       // ← setPower
                    } else {
                        trackTag();
                    }
                    break;

                // ─── LOST_WAITING ──────────────────────────────────
                // Tag perdu : on attend 3s avant de reprendre le scan
                case LOST_WAITING:
                    blinkLed(LED_ROUGE);
                    turretServo.setPower(CRSERVO_STOP);           // ← setPower

                    if (tagVisible) {
                        // Tag retrouvé avant le délai → reprendre le suivi
                        state = TurretState.TRACKING;
                        light.setPosition(LED_VERT);
                    } else if (lostTimer.milliseconds() >= LOST_TAG_TIMEOUT_MS) {
                        // 3 secondes écoulées → reprendre le scan
                        state = TurretState.SCANNING;
                        blinkTimer.reset();
                    }
                    break;
            }

            // 3) Télémétrie
            displayTelemetry();
            telemetry.update();
        }

        // ── Arrêt propre ─────────────────────────────────────────
        turretServo.setPower(CRSERVO_STOP);
        light.setPosition(LED_OFF);
        limelight.stop();
    }

    // ═══════════════════════════════════════════════════════════════
    //  MÉTHODES INTERNES
    // ═══════════════════════════════════════════════════════════════

    /**
     * Lit les données Limelight.
     * Met à jour : tagVisible et tagTx (offset horizontal en degrés).
     */
    private void updateLimelightData() {
        LLResult result = limelight.getLatestResult();
        tagVisible = false;
        tagTx      = 0.0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    if (TARGET_TAG_ID == -1 || tag.getFiducialId() == TARGET_TAG_ID) {
                        tagVisible = true;
                        tagTx      = tag.getTargetXDegrees();
                        break;
                    }
                }
            }
        }
    }

    /**
     * Contrôleur proportionnel (P) pour centrer le turret sur le tag.
     *
     * tagTx > 0 → tag à droite → puissance positive (tourne à droite)
     * tagTx < 0 → tag à gauche → puissance négative (tourne à gauche)
     * |tagTx| < DEADBAND → arrêt
     */
    private void trackTag() {
        if (Math.abs(tagTx) <= DEADBAND_DEGREES) {
            turretServo.setPower(CRSERVO_STOP);                   // ← setPower
            return;
        }

        // Calcul de la puissance proportionnelle
        double power = KP * tagTx;

        // Limiter entre -MAX et +MAX
        power = Math.max(-CRSERVO_MAX_POWER, Math.min(CRSERVO_MAX_POWER, power));

        turretServo.setPower(power);                              // ← setPower
    }

    /**
     * Fait clignoter la LED entre la couleur donnée et LED_OFF.
     */
    private void blinkLed(double color) {
        if (blinkTimer.milliseconds() >= BLINK_PERIOD_MS) {
            blinkState = !blinkState;
            blinkTimer.reset();
        }
        light.setPosition(blinkState ? color : LED_OFF);
    }

    /**
     * Télémétrie Driver Station
     */
    private void displayTelemetry() {
        telemetry.addLine("════ BRICKMAKERS 31563 ════");
        telemetry.addData("État Turret",  state.toString());
        telemetry.addData("Tag visible",  tagVisible ? "✅ OUI" : "❌ NON");
        telemetry.addData("Tag TX (°)",   String.format("%.2f°", tagTx));
        telemetry.addData("Turret power", String.format("%.3f", turretServo.getPower()));
        telemetry.addData("LED position", String.format("%.3f", light.getPosition()));

        if (state == TurretState.LOST_WAITING) {
            double restant = (LOST_TAG_TIMEOUT_MS - lostTimer.milliseconds()) / 1000.0;
            telemetry.addData("⏳ Reprise scan dans", String.format("%.1f sec", Math.max(0.0, restant)));
        }

        telemetry.addLine("───────────────────────────");
        telemetry.addData("Pipeline LL",  LIMELIGHT_PIPELINE);
        telemetry.addData("Tag ID cible", TARGET_TAG_ID == -1 ? "Tous" : "#" + TARGET_TAG_ID);
    }
}