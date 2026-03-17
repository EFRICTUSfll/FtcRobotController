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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * ════════════════════════════════════════════════════════════
 *  BRICKMAKERS — Team 31563
 *  NATIONAL LYON — TeleOp Final
 * ════════════════════════════════════════════════════════════
 *
 *  CONTRÔLES :
 *    Stick droit       → avancer / reculer / strafe
 *    Stick gauche X    → rotation
 *    Dpad haut/bas     → vitesse +/-
 *    LB                → ramassage toggle
 *    X (croix)         → montage toggle
 *    Triangle          → shooter ON/OFF (seulement si tag visible)
 *    Cercle            → monter les balles (seulement si shooter PRÊT)
 *    L2 / R2           → turret manuel gauche / droite
 *    R1                → bascule turret AUTO ↔ MANUEL
 *
 *  LED :
 *    Bleu   → attente, pas de tag
 *    Jaune  → tag visible, shooter pas actif
 *    Rouge  → shooter actif, monte en vitesse
 *    Vert   → shooter PRÊT → appuie Cercle
 *    Orange → ramassage actif
 *
 * ════════════════════════════════════════════════════════════
 *  SYSTÈME DE TIR — CALCUL CONTINU PAR DISTANCE
 * ════════════════════════════════════════════════════════════
 *
 *  La Limelight mesure ty (angle vertical du tag).
 *  La distance est calculée par trigonométrie :
 *
 *    distance = (H_CAMERA - H_TAG) / tan(ANGLE_CAMERA + tyCorrige)
 *
 *  Puis interpolée dans la table CALIBRATION (7 points) pour
 *  obtenir l'angle servo et la vitesse exacte à cette distance.
 *
 *  Caméra physiquement à l'envers (texte Limelight inversé) :
 *    txCorrige = -txBrut  (axe horizontal miroir)
 *    tyCorrige = -tyBrut  (axe vertical miroir)
 *
 * ════════════════════════════════════════════════════════════
 *  CALIBRATION — COMMENT REMPLIR LA TABLE :
 *
 *    1. Place le robot à exactement la distance indiquée
 *    2. Active shooter (Triangle), regarde la DS : "S : xxx/yyy"
 *    3. Ajuste la vitesse dans CALIBRATION jusqu'à bonne trajectoire
 *    4. Ajuste l'angle (angleServo) si la balle monte trop ou pas assez
 *    5. Répète pour chaque distance
 *    Le code interpole automatiquement entre les points.
 *
 *  CENTRER LE TURRET :
 *    Tag en face → turret se stabilise → lis TX sur DS
 *    → mets cette valeur dans TX_OFFSET_DEG
 *
 *  MAUVAIS SENS TURRET :
 *    Dans initHardware() ajouter :
 *    servoTurret.setDirection(DcMotorSimple.Direction.REVERSE);
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "NATIONAL LYON", group = "National")
public class MaxLyon extends LinearOpMode {

    // ════════════════════════════════════════════════════════
    //  HARDWARE
    // ════════════════════════════════════════════════════════

    public Servo      light;
    DcMotor           moteurAvantGauche, moteurAvantDroit;
    DcMotor           moteurArriereGauche, moteurArriereDroit;
    DcMotorEx         shooter;
    DcMotor           intakeMoteur;
    DcMotor           montageGauche, montageDroit;
    CRServo           servoRamassageDroit, servoRamassageGauche;
    CRServo           servoMoteurRamassageBalle;
    CRServo           servoTurret;
    Servo             servoAngleShooter;
    IMU               imu;
    Limelight3A       limelight;
    GoBildaPinpointDriver pinpoint;

    // ════════════════════════════════════════════════════════
    //  GÉOMÉTRIE CAMÉRA
    //  ⚙️  Vérifie avec un mètre ruban et une app niveau
    // ════════════════════════════════════════════════════════

    /** Hauteur de la Limelight depuis le sol (cm) */
    private static final double H_CAMERA     = 36.0;

    /** Hauteur du centre du tag depuis le sol (cm) — à mesurer sur le terrain */
    private static final double H_TAG        = 15.0;

    /** Inclinaison de la caméra vers le bas (degrés) */
    private static final double ANGLE_CAMERA = 17.0;

    // ════════════════════════════════════════════════════════
    //  TABLE DE CALIBRATION
    //
    //  { distanceCm, angleServo, vitesseTicks/s }
    //
    //  angleServo : 0.0 = 0° / 0.306 = 110° MAX (ne jamais dépasser)
    //  vitesseTicks : ticks/s encodeur shooter
    //
    //  Le code interpole linéairement entre chaque point.
    //  Hors plage → valeur de l'extrême la plus proche.
    // ════════════════════════════════════════════════════════

    private static final double[][] CALIBRATION = {
            //  { distCm,  angleServo,  vitesseTicks }
            {  20.0,   0.30,    900.0 },   //  20 cm
            {  40.0,   0.27,   1150.0 },   //  40 cm
            {  70.0,   0.24,   1500.0 },   //  70 cm
            { 100.0,   0.21,   1850.0 },   // 100 cm
            { 140.0,   0.17,   2250.0 },   // 140 cm
            { 180.0,   0.12,   2700.0 },   // 180 cm
            { 220.0,   0.07,   3200.0 },   // 220 cm
    };

    private static final double SHOOTER_TOLERANCE = 40.0; // ticks/s

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
    private static final double SHOOTER_I =  5.1;
    private static final double SHOOTER_D =  0.5;
    private static final double SHOOTER_F = 10.5;

    // ════════════════════════════════════════════════════════
    //  TURRET
    //  ⚙️  TX_OFFSET_DEG : seul réglage pour centrer le turret
    // ════════════════════════════════════════════════════════

    private static final double TX_OFFSET_DEG    =  0.0;
    private static final double KP_TURRET        =  0.025;
    private static final double KD_TURRET        =  0.003;
    private static final double DEADBAND_TURRET  =  2.5;
    private static final double MIN_POWER_TURRET =  0.07;
    private static final double MAX_POWER_TURRET =  0.60;

    private double      dernierTx      = 0.0;
    private ElapsedTime tempsTurret    = new ElapsedTime();
    private boolean     modeAutoTurret = true;
    private boolean     tagVisible     = false;

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
    private ElapsedTime debounceR1        = new ElapsedTime();
    private ElapsedTime debounceMonter    = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  BOUCLE PRINCIPALE
    // ════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        initHardware();
        configurerPID();

        light.setPosition(LED_BLEU);
        telemetry.addLine("Brickmakers 31563");
        telemetry.addLine("R1 = turret AUTO/MANUEL");
        telemetry.update();

        waitForStart();
        tempsTurret.reset();

        while (opModeIsActive()) {

            if (debounceR1.milliseconds() > 300 && gamepad1.right_bumper) {
                modeAutoTurret = !modeAutoTurret;
                if (!modeAutoTurret) servoTurret.setPower(0);
                debounceR1.reset();
            }

            gestionVitesse();
            deplacementFluide();
            gestionTurretEtTir();
            gestionShooter();
            gestionMonterBalles();
            gestionRamassage();
            gestionMontage();
            mettreAJourLED();

            // Télémétrie MINIMALE — 3 lignes
            telemetry.addData("T", modeAutoTurret ? "AUTO" : "MAN");
            telemetry.addData("D", tagVisible ? String.format("%.0fcm", distanceCourante) : "-");
            telemetry.addData("S", shooterActif
                    ? (shooterPret() ? "PRET!" : String.format("%.0f/%.0f", shooter.getVelocity(), vitesseCourante))
                    : "OFF");
            telemetry.update();
        }

        arret();
    }

    // ════════════════════════════════════════════════════════
    //  TURRET + DISTANCE + INTERPOLATION
    //  Un seul appel Limelight par loop — pas de double lecture
    // ════════════════════════════════════════════════════════

    private void gestionTurretEtTir() {
        float trigG = gamepad1.left_trigger;
        float trigD = gamepad1.right_trigger;

        // Manuel L2/R2 — priorité absolue
        if (trigD > 0.1 || trigG > 0.1) {
            servoTurret.setPower(trigD > 0.1 ? trigD * 0.5 : -trigG * 0.5);
            dernierTx  = 0;
            tagVisible = false;
            return;
        }

        // Lecture Limelight — une seule fois par loop
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > 300) {
            servoTurret.setPower(0);
            tagVisible = false;
            dernierTx  = 0;
            return;
        }

        FiducialResult tag = trouverTag(result);
        if (tag == null) {
            servoTurret.setPower(0);
            tagVisible = false;
            dernierTx  = 0;
            return;
        }

        tagVisible = true;

        // Caméra inversée — miroir horizontal et vertical
        double txCorrige = -tag.getTargetXDegrees() - TX_OFFSET_DEG;
        double tyCorrige = -tag.getTargetYDegrees();

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

        // Angle servo appliqué en permanence (même shooter OFF — prêt à tirer)
        servoAngleShooter.setPosition(angleServoCourant);

        // Suivi turret AUTO
        if (!modeAutoTurret) { servoTurret.setPower(0); return; }

        double dt      = tempsTurret.seconds();
        double derivee = (dt > 0.001) ? (txCorrige - dernierTx) / dt : 0;
        dernierTx = txCorrige;
        tempsTurret.reset();

        double p = 0;
        if (Math.abs(txCorrige) > DEADBAND_TURRET) {
            p = KP_TURRET * txCorrige + KD_TURRET * derivee;
            if      (p > 0 && p <  MIN_POWER_TURRET) p =  MIN_POWER_TURRET;
            else if (p < 0 && p > -MIN_POWER_TURRET) p = -MIN_POWER_TURRET;
            p = clamp(p, -MAX_POWER_TURRET, MAX_POWER_TURRET);
        }
        servoTurret.setPower(p);
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
                double t = (distCm - CALIBRATION[i][0]) / (CALIBRATION[i+1][0] - CALIBRATION[i][0]);
                double angle   = CALIBRATION[i][1] + t * (CALIBRATION[i+1][1] - CALIBRATION[i][1]);
                double vitesse = CALIBRATION[i][2] + t * (CALIBRATION[i+1][2] - CALIBRATION[i][2]);
                return new double[]{ Math.min(angle, 0.306), vitesse };
            }
        }
        return new double[]{ CALIBRATION[0][1], CALIBRATION[0][2] };
    }

    // ════════════════════════════════════════════════════════
    //  SHOOTER
    // ════════════════════════════════════════════════════════

    private void configurerPID() {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void gestionShooter() {
        if (debounceShooter.milliseconds() > 150 && gamepad1.triangle) {
            // Allume seulement si tag visible, peut éteindre toujours
            if (tagVisible || shooterActif) shooterActif = !shooterActif;
            debounceShooter.reset();
        }
        if (!tagVisible) shooterActif = false; // sécurité
        shooter.setVelocity(shooterActif ? vitesseCourante : 0);
    }

    private boolean shooterPret() {
        return shooterActif
                && Math.abs(shooter.getVelocity() - vitesseCourante) < SHOOTER_TOLERANCE;
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
            montageGauche.setPower(-0.3);
            montageDroit.setPower(-0.3);
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
            montageGauche.setPower(0.7);
            montageDroit.setPower(0.7);
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