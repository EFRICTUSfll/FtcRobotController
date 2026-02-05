package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MAX")
public class MaxRobotFTC extends LinearOpMode {
    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;
    DcMotorEx shooter;  // ← CHANGÉ en DcMotorEx pour le PID
    DcMotor montageMoteur;

    CRServo servoRamassageBalle;
    CRServo servoMoteurRamassageBalle;
    CRServo servoMontageGauche;
    CRServo servoTurret;
    Servo servoAngleShooter;

    IMU imu;

    private double vitesseDeplacement = 0.7;

    private boolean shooterActif = false;
    private boolean estRamassageActif = false;
    private boolean montageActif = false;

    private int positionAngleShooter = 1;  // Position actuelle : 1, 2, ou 3

    // ===================================
    // CONSTANTES PID SHOOTER (À AJUSTER)
    // ===================================
    private static final double SHOOTER_P = 10.0;   // Proportionnel
    private static final double SHOOTER_I = 0.1;    // Intégral
    private static final double SHOOTER_D = 0.5;    // Dérivé
    private static final double SHOOTER_F = 12.5;   // Feed-forward (compensation friction)

    private static final double SHOOTER_VITESSE_CIBLE = -2000.0;  // Vitesse en ticks/sec
    private static final double SHOOTER_TOLERANCE = 30.0;         // Tolérance en ticks/sec

    // ===================================
    // CONSTANTES SERVO ANGLE (POSITIONS EXACTES)
    // ===================================
    private static final double ANGLE_POSITION_1 = 1.0;    // Position basse
    private static final double ANGLE_POSITION_2 = 0.5;    // Position moyenne
    private static final double ANGLE_POSITION_3 = 0.0;    // Position haute

    // ===================================
    // Augmente la fluidité du robot
    // ===================================
    private double puissanceAvantGaucheActuelle = 0;
    private double puissanceAvantDroitActuelle = 0;
    private double puissanceArriereGaucheActuelle = 0;
    private double puissanceArriereDroitActuelle = 0;

    private final double TAUX_ACCELERATION = 0.25;
    private final double TAUX_DECELERATION = 0.35;
    private final double SEUIL_MORT = 0.05;

    private ElapsedTime tempsLoop = new ElapsedTime();
    private ElapsedTime tempsDebounceVitesse = new ElapsedTime();
    private ElapsedTime tempsDebounceRamassage = new ElapsedTime();
    private ElapsedTime tempsDebounceShooter = new ElapsedTime();
    private ElapsedTime tempsDebonceMontage = new ElapsedTime();
    private ElapsedTime tempsDebounceAngleShooter = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        configurerPIDShooter();  // ← NOUVELLE FONCTION
        waitForStart();

        tempsLoop.reset();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacementFluide();
            gestionRamassage();
            rotationTurret();
            gestionMontage();
            gestionShooterPID();      // ← CHANGÉ
            gestionAngleShooter();

            telemetry.update();
        }

        arretMoteurs();
    }

    // ===================================
    // CONFIGURATION PID DU SHOOTER
    // ===================================
    private void configurerPIDShooter() {
        // Définir les coefficients PIDF
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );

        // Appliquer les coefficients au moteur
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Mode velocity control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("PID Shooter", "Configuré: P=%.1f, I=%.1f, D=%.1f, F=%.1f",
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.update();
    }

    // ===================================
    // GESTION SHOOTER AVEC PID
    // ===================================
    private void gestionShooterPID() {
        // Debounce : on ne lit le bouton que toutes les 150ms
        if (tempsDebounceShooter.milliseconds() > 150) {
            if (gamepad1.triangle) {
                shooterActif = !shooterActif;
                tempsDebounceShooter.reset();
            }
        }

        if (shooterActif) {
            // Utiliser setVelocity pour le contrôle PID précis
            shooter.setVelocity(SHOOTER_VITESSE_CIBLE);

            // Vérifier si on est à la vitesse cible
            double vitesseActuelle = shooter.getVelocity();
            boolean aLaVitesse = Math.abs(vitesseActuelle - SHOOTER_VITESSE_CIBLE) < SHOOTER_TOLERANCE;

            telemetry.addData("Shooter", "ACTIF");
            telemetry.addData("Vitesse Cible", "%.0f ticks/sec", SHOOTER_VITESSE_CIBLE);
            telemetry.addData("Vitesse Actuelle", "%.0f ticks/sec", vitesseActuelle);
            telemetry.addData("Erreur", "%.0f ticks/sec", SHOOTER_VITESSE_CIBLE - vitesseActuelle);
            telemetry.addData("Prêt à tirer", aLaVitesse ? "OUI ✓" : "NON...");
        } else {
            shooter.setVelocity(0);
            telemetry.addData("Shooter", "ARRÊTÉ");
        }
    }

    // ===================================
    // GESTION ANGLE SHOOTER (CORRIGÉ)
    // ===================================
    private void gestionAngleShooter() {
        // Debounce : on ne lit le bouton cercle que toutes les 200ms
        if (tempsDebounceAngleShooter.milliseconds() > 200) {
            if (gamepad1.circle) {
                // Passer à la position suivante (cycle 1→2→3→1)
                positionAngleShooter++;
                if (positionAngleShooter > 3) {
                    positionAngleShooter = 1;
                }
                tempsDebounceAngleShooter.reset();
            }
        }

        // Appliquer la position au servo (CORRIGÉ)
        double positionCible = ANGLE_POSITION_1;

        switch (positionAngleShooter) {
            case 1:
                positionCible = ANGLE_POSITION_1;
                break;
            case 2:
                positionCible = ANGLE_POSITION_2;
                break;
            case 3:
                positionCible = ANGLE_POSITION_3;
                break;
        }

        servoAngleShooter.setPosition(positionCible);

        telemetry.addData("Angle Shooter", "Position %d/3 (%.2f)", positionAngleShooter, positionCible);
    }

    private static final double VITESSE_TURRET = 0.5; // vitesse de base 0,4

    private void rotationTurret() {
        float gachetteGauche = gamepad1.left_trigger;
        float gachetteDroit  = gamepad1.right_trigger;

        if (gachetteDroit > 0.1) {
            servoTurret.setPower(gachetteDroit * VITESSE_TURRET);
        } else if (gachetteGauche > 0.1) {
            servoTurret.setPower(-gachetteGauche * VITESSE_TURRET);
        } else {
            servoTurret.setPower(0);
        }

        telemetry.addData("Turret", "L2: %.2f | R2: %.2f", gachetteGauche, gachetteDroit);
    }

    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
        shooter.setVelocity(0);  // ← CHANGÉ pour PID
        montageMoteur.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoRamassageBalle.setPower(0);
        servoMontageGauche.setPower(0);
    }

    private void gestionMontage() {
        if (tempsDebonceMontage.milliseconds() > 150) {
            if (gamepad1.x) {
                montageActif = !montageActif;
                tempsDebonceMontage.reset();
            }
        }

        if (montageActif) {
            servoMontageGauche.setPower(-0.7);
            montageMoteur.setPower(-0.7);
        } else {
            servoMontageGauche.setPower(0.0);
            montageMoteur.setPower(0.0);
        }
    }

    private void gestionRamassage() {
        if (tempsDebounceRamassage.milliseconds() > 200) {
            if (gamepad1.left_bumper) {
                estRamassageActif = !estRamassageActif;
                tempsDebounceRamassage.reset();
            }
        }

        if (estRamassageActif) {
            servoMoteurRamassageBalle.setPower(-1.0);
            servoRamassageBalle.setPower(1.0);
        } else {
            servoMoteurRamassageBalle.setPower(0.0);
            servoRamassageBalle.setPower(0.0);
        }
    }

    private void gestionVitesseDeplacement() {
        if (tempsDebounceVitesse.milliseconds() > 300) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                vitesseDeplacement += 0.1;
                if (vitesseDeplacement > 1.0) vitesseDeplacement = 1.0;
                tempsDebounceVitesse.reset();
            }

            if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                vitesseDeplacement -= 0.1;
                if (vitesseDeplacement < 0.1) vitesseDeplacement = 0.1;
                tempsDebounceVitesse.reset();
            }
        }

        telemetry.addData("Vitesse Deplacement", "%.0f%%", vitesseDeplacement * 100);
    }

    private void initialisationDuRobot() {
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");  // ← DcMotorEx pour PID
        montageMoteur = hardwareMap.get(DcMotor.class, "montage");

        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageBalle = hardwareMap.get(CRServo.class, "ramassage2");
        servoTurret = hardwareMap.get(CRServo.class, "rotationShooter");
        servoMontageGauche = hardwareMap.get(CRServo.class, "montageGauche");
        servoAngleShooter = hardwareMap.get(Servo.class, "angleShooter");

        // Position initiale de l'angle shooter : position 1
        servoAngleShooter.setPosition(ANGLE_POSITION_1);

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        montageMoteur.setDirection(DcMotor.Direction.REVERSE);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // PID activé
        montageMoteur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moteurAvantGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurAvantDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    // ===================================
    // DÉPLACEMENT FLUIDE (INCHANGÉ)
    // ===================================
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

        telemetry.addData("Vitesse Max", "%.0f%%", vitesseDeplacement * 100);
    }

    private double appliquerRampe(double valeurActuelle, double valeurCible) {
        if (valeurCible == 0 && Math.abs(valeurActuelle) < 0.05) {
            return 0;
        }

        double difference = valeurCible - valeurActuelle;
        double taux = (valeurCible == 0) ? TAUX_DECELERATION : TAUX_ACCELERATION;

        return valeurActuelle + (difference * taux);
    }

    private double appliquerZoneMorte(double valeur) {
        if (Math.abs(valeur) < SEUIL_MORT) {
            return 0;
        }
        return valeur;
    }
}