package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * AprilTagTurretTracker
 * ─────────────────────
 * Système de suivi AprilTag pour turret servo goBILDA + Limelight 3A.
 *
 * Logique :
 *   1. Pas de tag détecté → le servo SWEEP (balaye) pour chercher
 *   2. Tag détecté → PID pour centrer le tag (tx → 0)
 *   3. Tag perdu → attendre un court timeout, puis reprendre le sweep
 *
 * Conçu pour FTC DECODE 2025-2026.
 * Les AprilTags sont sur les goals pour l'aide au ciblage.
 */
public class LimeLightTest {

    // ══════════════════════════════════════════
    //  ÉTATS DE LA MACHINE
    // ══════════════════════════════════════════
    public enum State {
        IDLE,       // Système éteint
        SCANNING,   // Balayage pour trouver un tag
        TRACKING,   // Suivi actif d'un tag
        LOST        // Tag perdu, en attente avant de rescanner
    }

    private State currentState = State.IDLE;

    // ══════════════════════════════════════════
    //  HARDWARE
    // ══════════════════════════════════════════
    private final Limelight3A limelight;
    private final ServoImplEx turretServo;

    // ══════════════════════════════════════════
    //  CONSTANTES SERVO goBILDA
    // ══════════════════════════════════════════
    // goBILDA 2000 Series : 300° avec PWM étendu (500-2500μs)
    private static final double SERVO_MIN     = 0.0;
    private static final double SERVO_MAX     = 1.0;
    private static final double SERVO_CENTER  = 0.5;
    private static final double SERVO_RANGE_DEG = 300.0; // degrés sur toute la plage

    // ══════════════════════════════════════════
    //  PARAMÈTRES DE SCANNING (BALAYAGE)
    // ══════════════════════════════════════════
    private double scanPosition   = SERVO_CENTER; // Position actuelle du scan
    private double scanDirection  = 1.0;          // +1 = droite, -1 = gauche
    private double scanSpeed      = 0.005;        // Incrément par cycle (~1.5°/cycle)
    private double scanMin        = 0.05;         // Limite gauche du scan
    private double scanMax        = 0.95;         // Limite droite du scan

    // ══════════════════════════════════════════
    //  PARAMÈTRES PID POUR LE TRACKING
    // ══════════════════════════════════════════
    private double kP = 0.015;   // Gain proportionnel
    private double kI = 0.0;     // Gain intégral
    private double kD = 0.002;   // Gain dérivé

    private double integralSum   = 0.0;
    private double lastError     = 0.0;
    private double integralMax   = 0.1;  // Anti-windup

    // Zone morte : si |tx| < deadzone → on considère que c'est centré
    private double deadzone = 1.0; // degrés

    // ══════════════════════════════════════════
    //  PARAMÈTRES DE PERTE DE CIBLE
    // ══════════════════════════════════════════
    private final ElapsedTime lostTimer = new ElapsedTime();
    private double lostTimeoutMs = 500; // ms avant de passer en SCANNING

    // ══════════════════════════════════════════
    //  DONNÉES DE TRACKING
    // ══════════════════════════════════════════
    private double currentTx = 0.0;  // Offset horizontal du tag (degrés)
    private double currentTy = 0.0;  // Offset vertical
    private double currentTa = 0.0;  // Aire du tag (%)
    private int    currentTagId = -1; // ID du tag suivi
    private boolean targetVisible = false;

    // Pipeline Limelight pour les AprilTags
    private int aprilTagPipeline = 0;

    // ══════════════════════════════════════════
    //  CONSTRUCTEUR
    // ══════════════════════════════════════════
    /**
     * @param hardwareMap   Le HardwareMap du robot
     * @param servoName     Nom du servo dans la config (ex: "turret_servo")
     * @param limelightName Nom du Limelight dans la config (ex: "limelight")
     * @param pipeline      Numéro du pipeline AprilTag sur le Limelight
     */
    public LimeLightTest(HardwareMap hardwareMap,
                         String servoName,
                         String limelightName,
                         int pipeline) {

        // Initialiser le servo avec PWM étendu pour les 300° complets
        turretServo = hardwareMap.get(ServoImplEx.class, servoName);
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Initialiser le Limelight
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        this.aprilTagPipeline = pipeline;

        // Centrer le servo
        turretServo.setPosition(SERVO_CENTER);
        scanPosition = SERVO_CENTER;
    }

    // ══════════════════════════════════════════
    //  DÉMARRAGE / ARRÊT
    // ══════════════════════════════════════════

    /** Démarre le Limelight et active le tracking */
    public void start() {
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(aprilTagPipeline);
        limelight.start();
        resetPID();
        currentState = State.SCANNING;
    }

    /** Arrête tout et recentre le servo */
    public void stop() {
        currentState = State.IDLE;
        limelight.stop();
        turretServo.setPosition(SERVO_CENTER);
        scanPosition = SERVO_CENTER;
    }

    // ══════════════════════════════════════════
    //  BOUCLE PRINCIPALE — appeler dans loop()
    // ══════════════════════════════════════════

    /**
     * Méthode à appeler à chaque itération de loop().
     * Gère toute la logique : lecture vision → décision → commande servo.
     */
    public void update() {
        if (currentState == State.IDLE) return;

        // ── Étape 1 : Lire les données du Limelight ──
        readLimelight();

        // ── Étape 2 : Machine à états ──
        switch (currentState) {

            case SCANNING:
                if (targetVisible) {
                    // Tag trouvé ! → passer en tracking
                    currentState = State.TRACKING;
                    resetPID();
                    // Synchroniser la position de scan avec la position actuelle
                    scanPosition = turretServo.getPosition();
                } else {
                    // Continuer le balayage
                    doScan();
                }
                break;

            case TRACKING:
                if (targetVisible) {
                    // Tag toujours visible → correction PID
                    doTracking();
                } else {
                    // Tag perdu → démarrer le timer
                    currentState = State.LOST;
                    lostTimer.reset();
                }
                break;

            case LOST:
                if (targetVisible) {
                    // Tag retrouvé !
                    currentState = State.TRACKING;
                    resetPID();
                } else if (lostTimer.milliseconds() > lostTimeoutMs) {
                    // Timeout écoulé → reprendre le scan
                    currentState = State.SCANNING;
                    // On commence le scan depuis la dernière position connue
                    scanPosition = turretServo.getPosition();
                }
                // Pendant le timeout, on ne bouge pas (on reste sur la dernière position)
                break;
        }
    }

    // ══════════════════════════════════════════
    //  LECTURE DU LIMELIGHT
    // ══════════════════════════════════════════

    private void readLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Vérifier s'il y a des fiducials (AprilTags)
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Prendre le premier tag détecté (ou le plus gros)
                LLResultTypes.FiducialResult bestTag = fiducials.get(0);

                currentTx       = result.getTx();
                currentTy       = result.getTy();
                currentTa       = result.getTa();
                currentTagId    = bestTag.getFiducialId();
                targetVisible   = true;
                return;
            }
        }

        // Aucun tag valide
        targetVisible = false;
    }

    // ══════════════════════════════════════════
    //  BALAYAGE (SCANNING)
    // ══════════════════════════════════════════

    private void doScan() {
        // Incrémenter la position dans la direction actuelle
        scanPosition += scanSpeed * scanDirection;

        // Inverser la direction aux limites
        if (scanPosition >= scanMax) {
            scanPosition = scanMax;
            scanDirection = -1.0;
        } else if (scanPosition <= scanMin) {
            scanPosition = scanMin;
            scanDirection = 1.0;
        }

        turretServo.setPosition(scanPosition);
    }

    // ══════════════════════════════════════════
    //  SUIVI PID (TRACKING)
    // ══════════════════════════════════════════

    private void doTracking() {
        // L'erreur est tx : degrés de déviation horizontale
        // tx > 0 → tag à droite → servo doit tourner à droite (position ↑)
        // tx < 0 → tag à gauche → servo doit tourner à gauche (position ↓)
        double error = currentTx;

        // Si dans la zone morte, ne rien faire
        if (Math.abs(error) < deadzone) {
            lastError = error;
            return;
        }

        // PID
        integralSum += error;
        integralSum = Range.clip(integralSum, -integralMax, integralMax); // Anti-windup

        double derivative = error - lastError;

        double correction = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Appliquer la correction à la position actuelle du servo
        double currentPos = turretServo.getPosition();
        double newPos = currentPos + correction;

        // Clamp entre les limites du servo
        newPos = Range.clip(newPos, SERVO_MIN, SERVO_MAX);

        turretServo.setPosition(newPos);

        // Mettre à jour pour le prochain cycle
        lastError = error;

        // Synchroniser la position de scan (au cas où on repasse en SCANNING)
        scanPosition = newPos;
    }

    private void resetPID() {
        integralSum = 0.0;
        lastError   = 0.0;
    }

    // ══════════════════════════════════════════
    //  GETTERS (pour la télémétrie)
    // ══════════════════════════════════════════

    public State   getState()         { return currentState; }
    public boolean isTargetVisible()  { return targetVisible; }
    public double  getTx()            { return currentTx; }
    public double  getTy()            { return currentTy; }
    public double  getTa()            { return currentTa; }
    public int     getTagId()         { return currentTagId; }
    public double  getServoPosition() { return turretServo.getPosition(); }

    /** Convertit la position servo (0-1) en degrés (0-300°) */
    public double getServoDegrees() {
        return turretServo.getPosition() * SERVO_RANGE_DEG;
    }

    // ══════════════════════════════════════════
    //  SETTERS (pour calibration en live)
    // ══════════════════════════════════════════

    /** Ajuste les gains PID */
    public void setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    /** Ajuste la zone morte en degrés */
    public void setDeadzone(double degrees) {
        this.deadzone = degrees;
    }

    /** Ajuste la vitesse de scan (incrément servo par cycle) */
    public void setScanSpeed(double speed) {
        this.scanSpeed = Math.abs(speed);
    }

    /** Ajuste les limites du scan (positions servo 0.0-1.0) */
    public void setScanLimits(double min, double max) {
        this.scanMin = Range.clip(min, SERVO_MIN, SERVO_MAX);
        this.scanMax = Range.clip(max, SERVO_MIN, SERVO_MAX);
    }

    /** Ajuste le timeout de perte de cible en ms */
    public void setLostTimeout(double ms) {
        this.lostTimeoutMs = ms;
    }

    /** Change le pipeline Limelight */
    public void setPipeline(int pipeline) {
        this.aprilTagPipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }
}