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

import java.util.List;

/**
 * ════════════════════════════════════════════════════════════
 *  MAX — TeleOp National
 *
 *  Fonctionnalités :
 *   • Déplacement Mecanum fluide (rampe acc/dec)
 *   • Turret CRS → suivi AprilTag via Limelight 3A (PID)
 *     - L2/R2 : contrôle manuel du turret (override)
 *     - Le suivi automatique reprend si aucune gâchette n'est pressée
 *   • Shooter PIDF vitesse (GoBilda Yellow Jacket)
 *     - 10 presets angle + vitesse de 10 cm à 200 cm du panier
 *   • Servo angle shooter 360° GoBilda → MODE ANGLE
 *   • Odométrie GoBilda Pinpoint → stabilisation à l'arrêt
 *   • LED GoBilda (light) → indicateur d'état
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "MAX-Final", group = "National")
public class MaxF extends LinearOpMode {

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
    private static final double SHOOTER_TOLERANCE = 10.0; // ticks/s d'écart accepté

    // ════════════════════════════════════════════════════════
    //  PRESETS DE TIR
    //
    //  Structure : { angleServo (0.0–1.0), vitesseShooter (ticks/s) }
    //  angleServo : 0.0 = 0°, 1.0 = 300° (servo GoBilda mode ANGLE)
    //               → tu corriges ces valeurs après tes tests terrain
    //
    //  Distances estimées du panier (tu ajustes à la main) :
    //   Preset 1  →  ~10  cm
    //   Preset 2  →  ~30  cm
    //   Preset 3  →  ~50  cm
    //   Preset 4  →  ~70  cm
    //   Preset 5  →  ~90  cm
    //   Preset 6  →  ~110 cm
    //   Preset 7  →  ~130 cm
    //   Preset 8  →  ~150 cm
    //   Preset 9  →  ~175 cm
    //   Preset 10 →  ~200 cm
    // ════════════════════════════════════════════════════════

    private static final double[][] PRESETS_TIR = {
            // { angleServo,  vitesse ticks/s }
            { 0.95,  800.0  },   // Preset 1  — ~10  cm du panier
            { 0.88, 1000.0  },   // Preset 2  — ~30  cm du panier
            { 0.80, 1200.0  },   // Preset 3  — ~50  cm du panier
            { 0.72, 1500.0  },   // Preset 4  — ~70  cm du panier
            { 0.63, 1800.0  },   // Preset 5  — ~90  cm du panier
            { 0.55, 2100.0  },   // Preset 6  — ~110 cm du panier
            { 0.47, 2400.0  },   // Preset 7  — ~130 cm du panier
            { 0.38, 2750.0  },   // Preset 8  — ~150 cm du panier
            { 0.28, 3100.0  },   // Preset 9  — ~175 cm du panier
            { 0.18, 3500.0  },   // Preset 10 — ~200 cm du panier
    };

    private int presetActif = 0; // index 0–9

    // ════════════════════════════════════════════════════════
    //  TURRET — SUIVI APRILTAG (PID)
    //
    //  Servo GoBilda 360° Dual Mode → MODE CRS (continu)
    //  0.5 = stop | >0.5 = droite | <0.5 = gauche
    //
    //  Caméra Limelight inclinée entre 21° et 28° vers le bas.
    //  tx = 0  → tag centré horizontalement → ne rien faire
    //  tx > 0  → tag à droite → tourner à droite
    //  tx < 0  → tag à gauche → tourner à gauche
    // ════════════════════════════════════════════════════════

    /** ID du tag à suivre. -1 = suit n'importe quel tag visible. */
    private static final int    TURRET_TAG_ID       = -1;

    /** Gain proportionnel : puissance = KP_TURRET * tx */
    private static final double KP_TURRET           = 0.028;

    /** Gain dérivé : amortit les oscillations */
    private static final double KD_TURRET           = 0.004;

    /** Zone morte : en dessous de X degrés, on ne corrige pas */
    private static final double TURRET_DEADBAND_DEG = 2.0;

    /** Puissance minimale pour vaincre les frottements mécaniques */
    private static final double TURRET_MIN_POWER    = 0.07;

    /** Puissance maximale envoyée au servo turret */
    private static final double TURRET_MAX_POWER    = 0.65;

    // État interne PD turret
    private double derniereTxTurret = 0.0;
    private ElapsedTime tempsTurret = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  ODOMÉTRIE — STABILISATION À L'ARRÊT (Pinpoint)
    //
    //  Problème : le robot glisse/dérive quand on lâche le joystick.
    //  Solution : quand les joysticks sont à 0, on lit la vitesse
    //  mesurée par le Pinpoint et on applique une correction inverse
    //  proportionnelle pour contrer le glissement, sans affecter
    //  la vitesse de déplacement volontaire.
    // ════════════════════════════════════════════════════════

    /** Gain de la correction de stabilisation (seulement à l'arrêt) */
    private static final double KP_STABILISATION    = 0.4;

    /** Vitesse (mm/s) en dessous de laquelle on considère le robot arrêté */
    private static final double SEUIL_ARRET_MMS     = 15.0;

    /** Puissance max de correction de stabilisation (ne jamais dépasser) */
    private static final double MAX_CORRECTION_STAB = 0.20;

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT — RAMPE ACC/DEC
    // ════════════════════════════════════════════════════════

    private double vitesseDeplacement           = 0.7;
    private double puissanceAvantGaucheActuelle = 0;
    private double puissanceAvantDroitActuelle  = 0;
    private double puissanceArriereGaucheActuelle = 0;
    private double puissanceArriereDroitActuelle = 0;

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

    private ElapsedTime tempsLoop                = new ElapsedTime();
    private ElapsedTime tempsDebounceVitesse     = new ElapsedTime();
    private ElapsedTime tempsDebounceRamassage   = new ElapsedTime();
    private ElapsedTime tempsDebounceShooter     = new ElapsedTime();
    private ElapsedTime tempsDebonceMontage      = new ElapsedTime();
    private ElapsedTime tempsDebounceAngleShooter= new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  BOUCLE PRINCIPALE
    // ════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        configurerPIDShooter();

        light.setPosition(0.611); // LED : initialisation (bleu)
        waitForStart();
        tempsLoop.reset();
        tempsTurret.reset();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacementFluide();
            gestionTurret();
            gestionRamassage();
            gestionMontage();
            gestionShooterPID();
            gestionAngleShooter();

            // LED verte si shooter actif et à vitesse, orange sinon
            light.setPosition(shooterActif ? 0.772 : 0.388);

            telemetry.update();
        }

        arretMoteurs();
    }

    // ════════════════════════════════════════════════════════
    //  TURRET — SUIVI APRILTAG + OVERRIDE MANUEL
    // ════════════════════════════════════════════════════════

    /**
     * Priorité :
     *  1. Si L2 ou R2 pressé → contrôle manuel direct, suivi désactivé
     *  2. Sinon → suivi automatique via Limelight (PD)
     */
    private void gestionTurret() {
        float gachetteGauche = gamepad1.left_trigger;
        float gachetteDroit  = gamepad1.right_trigger;

        // ── Mode manuel (override) ────────────────────────
        if (gachetteDroit > 0.1 || gachetteGauche > 0.1) {
            double puissance = 0;
            if (gachetteDroit  > 0.1) puissance =  gachetteDroit  * 0.5;
            if (gachetteGauche > 0.1) puissance = -gachetteGauche * 0.5;
            servoTurret.setPower(puissance);
            derniereTxTurret = 0; // reset dérivée pour éviter un pic au retour auto
            return;
        }

        // ── Mode automatique : suivi AprilTag ─────────────
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getStaleness() > 300) {
            servoTurret.setPower(0);
            return;
        }

        FiducialResult tag = trouverTag(result);

        if (tag == null) {
            servoTurret.setPower(0);
            return;
        }

        // tx : degrés horizontaux depuis le centre de l'image
        // La caméra est inclinée (21–28°) mais ça n'affecte pas tx (axe horizontal)
        double tx = tag.getTargetXDegrees();

        // ── Calcul PD ─────────────────────────────────────
        double dt      = tempsTurret.seconds();
        double derivee = (dt > 0) ? (tx - derniereTxTurret) / dt : 0;
        derniereTxTurret = tx;
        tempsTurret.reset();

        double puissance = 0;

        if (Math.abs(tx) > TURRET_DEADBAND_DEG) {
            puissance = KP_TURRET * tx + KD_TURRET * derivee;

            // Puissance minimale pour vaincre les frottements
            if (puissance > 0 && puissance < TURRET_MIN_POWER)
                puissance = TURRET_MIN_POWER;
            else if (puissance < 0 && puissance > -TURRET_MIN_POWER)
                puissance = -TURRET_MIN_POWER;

            puissance = clamp(puissance, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        // CRServo : setPower(-1 à +1), pas setPosition
        servoTurret.setPower(puissance);

        telemetry.addData("Turret tx", "%.1f°  pwr=%.2f", tx, puissance);
    }

    // ════════════════════════════════════════════════════════
    //  SHOOTER — PIDF VITESSE
    // ════════════════════════════════════════════════════════

    private void configurerPIDShooter() {
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void gestionShooterPID() {
        // Triangle → toggle shooter
        if (tempsDebounceShooter.milliseconds() > 150 && gamepad1.triangle) {
            shooterActif = !shooterActif;
            tempsDebounceShooter.reset();
        }

        if (shooterActif) {
            double vitesseCible = PRESETS_TIR[presetActif][1];
            shooter.setVelocity(vitesseCible);

            double vitesseActuelle = shooter.getVelocity();
            boolean pret = Math.abs(vitesseActuelle - vitesseCible) < SHOOTER_TOLERANCE;
            telemetry.addData("Shooter", "Preset %d | %.0f→%.0f t/s | %s",
                    presetActif + 1, vitesseActuelle, vitesseCible, pret ? "PRÊT ✓" : "...");
        } else {
            shooter.setVelocity(0);
        }
    }

    // ════════════════════════════════════════════════════════
    //  ANGLE SHOOTER — SERVO 360° GOBILDA MODE ANGLE
    //
    //  Cercle → cycle parmi les 10 presets
    //  Le servo GoBilda 360° Torque Super Speed doit être en
    //  MODE ANGLE (pas CRS) pour tenir une position précise.
    //  setPosition(0.0–1.0) correspond à 0°–360°.
    // ════════════════════════════════════════════════════════

    private void gestionAngleShooter() {
        if (tempsDebounceAngleShooter.milliseconds() > 200 && gamepad1.circle) {
            presetActif = (presetActif + 1) % PRESETS_TIR.length;
            tempsDebounceAngleShooter.reset();
        }

        servoAngleShooter.setPosition(PRESETS_TIR[presetActif][0]);

        telemetry.addData("Angle Shooter", "Preset %d/10 → pos=%.2f",
                presetActif + 1, PRESETS_TIR[presetActif][0]);
    }

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT FLUIDE + STABILISATION PINPOINT
    // ════════════════════════════════════════════════════════

    public void deplacementFluide() {
        double forward = -gamepad1.right_stick_y;
        double right = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        forward = appliquerZoneMorte(forward);
        right = appliquerZoneMorte(right);
        rotate = appliquerZoneMorte(rotate);

        double frontLeftPowerCible = forward + right + rotate;
        double frontRightPowerCible = forward - right - rotate;
        double backRightPowerCible = forward + right - rotate;
        double backLeftPowerCible = forward - right + rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPowerCible), Math.abs(frontRightPowerCible)),
                Math.max(Math.abs(backLeftPowerCible), Math.abs(backRightPowerCible))
        );

        if (maxPower > 1.0) {
            frontLeftPowerCible /= maxPower;
            frontRightPowerCible /= maxPower;
            backLeftPowerCible /= maxPower;
            backRightPowerCible /= maxPower;
        }

        puissanceAvantGaucheActuelle = appliquerRampe(puissanceAvantGaucheActuelle, frontLeftPowerCible);
        puissanceAvantDroitActuelle = appliquerRampe(puissanceAvantDroitActuelle, frontRightPowerCible);
        puissanceArriereGaucheActuelle = appliquerRampe(puissanceArriereGaucheActuelle, backLeftPowerCible);
        puissanceArriereDroitActuelle = appliquerRampe(puissanceArriereDroitActuelle, backRightPowerCible);

        moteurAvantGauche.setPower(vitesseDeplacement * puissanceAvantGaucheActuelle);
        moteurAvantDroit.setPower(vitesseDeplacement * puissanceAvantDroitActuelle);
        moteurArriereGauche.setPower(vitesseDeplacement * puissanceArriereGaucheActuelle);
        moteurArriereDroit.setPower(vitesseDeplacement * puissanceArriereDroitActuelle);

        //telemetry.addData("Vitesse Max", "%.0f%%", vitesseDeplacement * 100);
    }

    // ════════════════════════════════════════════════════════
    //  AUTRES FONCTIONS (identiques à l'original)
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
            light.setPosition(.222);
            intakeMoteur.setPower(1);
            servoMoteurRamassageBalle.setPower(-1.0);
            servoRamassageGauche.setPower(-0.3);
            servoRamassageDroit.setPower(0.3);
            // Servos et Moteurs du montage qui tourne a contre-Sens pour que les balles ne montent pas
            montageGauche.setPower(-1.0);
            montageDroit.setPower(-1.0);

        } else {
            intakeMoteur.setPower(0.0);
            servoMoteurRamassageBalle.setPower(0.0);
        }
    }

    private void gestionMontage() {
        if (tempsDebonceMontage.milliseconds() > 400 && gamepad1.x) {
            montageActif = !montageActif;
            tempsDebonceMontage.reset();
        }
        if (montageActif) {
            servoRamassageGauche.setPower(0.3);
            servoRamassageDroit.setPower(-0.3);
            servoMoteurRamassageBalle.setPower(-0.6);
            montageGauche.setPower(1.0);
            montageDroit.setPower(1.0);
            intakeMoteur.setPower(1.0);
        } else {
            servoRamassageGauche.setPower(0.0);
            servoRamassageDroit.setPower(0.0);
            servoMoteurRamassageBalle.setPower(0.0);
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
        servoAngleShooter.setPosition(PRESETS_TIR[0][0]);
        limelight.stop();
    }

    // ════════════════════════════════════════════════════════
    //  INITIALISATION
    // ════════════════════════════════════════════════════════

    private void initialisationDuRobot() {
        // ── Moteurs drivetrain ─────────────────────────────
        moteurAvantDroit     = hardwareMap.get(DcMotor.class,   "avantDroit");
        moteurAvantGauche    = hardwareMap.get(DcMotor.class,   "avantGauche");
        moteurArriereDroit   = hardwareMap.get(DcMotor.class,   "dosDroit");
        moteurArriereGauche  = hardwareMap.get(DcMotor.class,   "dosGauche");
        shooter              = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMoteur         = hardwareMap.get(DcMotor.class,   "intake");
        montageGauche        = hardwareMap.get(DcMotor.class,   "montageG");
        montageDroit         = hardwareMap.get(DcMotor.class,   "montageD");

        // ── Servos ─────────────────────────────────────────
        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageDroit       = hardwareMap.get(CRServo.class, "ramassageD");
        servoRamassageGauche      = hardwareMap.get(CRServo.class, "ramassageG");
        servoTurret               = hardwareMap.get(CRServo.class, "turret");      // MODE CRS
        servoAngleShooter         = hardwareMap.get(Servo.class,   "angleShooter"); // MODE ANGLE
        light                     = hardwareMap.get(Servo.class,   "light");

        // ── Capteurs ───────────────────────────────────────
        imu      = hardwareMap.get(IMU.class,              "imu");
        limelight = hardwareMap.get(Limelight3A.class,     "limelight");
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, "odoComputer");

        // ── Directions moteurs ─────────────────────────────
        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intakeMoteur.setDirection(DcMotor.Direction.REVERSE);
        montageGauche.setDirection(DcMotor.Direction.FORWARD);
        montageDroit.setDirection(DcMotorSimple.Direction.REVERSE);

        // ── Modes moteurs ──────────────────────────────────
        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ── Comportement à l'arrêt ─────────────────────────
        moteurAvantGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurAvantDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── IMU ────────────────────────────────────────────
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        // ── Limelight ──────────────────────────────────────
        limelight.pipelineSwitch(0); // Pipeline 0 = AprilTag (Full 3D activé dans le web UI)
        limelight.setPollRateHz(100);
        limelight.start();

        // ── Pinpoint odométrie ─────────────────────────────
        // Adapte les offsets X/Y selon la position physique de ton Pinpoint sur le robot
        // X = offset latéral (mm), Y = offset longitudinal (mm) depuis le centre du robot

        /*pinpoint.setOffsets(0, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

         */

        // ── Position initiale servo ────────────────────────
        servoAngleShooter.setPosition(PRESETS_TIR[0][0]);
        light.setPosition(0.388);
    }

    // ════════════════════════════════════════════════════════
    //  UTILITAIRES
    // ════════════════════════════════════════════════════════

    /**
     * Cherche le FiducialResult correspondant à TURRET_TAG_ID.
     * Si TURRET_TAG_ID == -1, retourne le tag le plus centré (|tx| minimal).
     */
    private FiducialResult trouverTag(LLResult result) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        FiducialResult meilleur  = null;
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
