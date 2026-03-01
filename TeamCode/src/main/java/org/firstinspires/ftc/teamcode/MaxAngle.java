package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MAX")
public class MaxAngle extends LinearOpMode {
    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;
    DcMotorEx shooter;
    DcMotor intakeMoteur;
    DcMotor montageGauche;
    DcMotor montageDroit;


    CRServo servoRamassageDroit;
    CRServo servoRamassageGauche;
    CRServo servoMoteurRamassageBalle;
    CRServo servoTurret;
    Servo servoAngleShooter;

    IMU imu;

    private double vitesseDeplacement = 0.7;
    private boolean shooterActif = false;
    private boolean estRamassageActif = false;
    private boolean montageActif = false;

    private int positionAngleShooter = 1;

    // ===================================
    // CONSTANTES PID SHOOTER
    // ===================================
    private static final double SHOOTER_P = 15.0;
    private static final double SHOOTER_I = 5.1;
    private static final double SHOOTER_D = 0.5;
    private static final double SHOOTER_F = 10.5;

    // ===================================
    // VITESSES SHOOTER PAR POSITION (MODIFIÉ)
    // ===================================
    private static final double SHOOTER_VITESSE_POSITION_1 = 1100.0;  // Position Basse
    private static final double SHOOTER_VITESSE_POSITION_2 = 15000.0;  // Position Milieu
    private static final double SHOOTER_VITESSE_POSITION_3 = 2100.0;  // Position Haute

    private static final double SHOOTER_TOLERANCE = 10.0;

    // ===================================
    // Angle shooter - 3 Positions Possible
    // ===================================
    private static final double ANGLE_POSITION_1 = 1.0;
    private static final double ANGLE_POSITION_2 = 0.5;
    private static final double ANGLE_POSITION_3 = 0.0;

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
        configurerPIDShooter();
        waitForStart();

        tempsLoop.reset();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacementFluide();
            gestionRamassage();
            rotationTurret();
            gestionMontage();
            gestionShooterPID();
            gestionAngleShooter();

            telemetry.update();
        }
        arretMoteurs();
    }

    // ===================================
    // CONFIGURATION PID DU SHOOTER
    // ===================================
    private void configurerPIDShooter() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //telemetry.addData("PID Shooter", "Configuré: P=%.1f, I=%.1f, D=%.1f, F=%.1f",
        //        SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        //telemetry.update();
    }

    // ===================================
    // Shooter avec PIDF
    // ===================================
    private void gestionShooterPID() {
        if (tempsDebounceShooter.milliseconds() > 150) {
            if (gamepad1.triangle) {
                shooterActif = !shooterActif;
                tempsDebounceShooter.reset();
            }
        }

        if (shooterActif) {
            // ===== SÉLECTION DE LA VITESSE SELON LA POSITION =====
            double vitesseCible = SHOOTER_VITESSE_POSITION_1;  // Par défaut

            switch (positionAngleShooter) {
                case 1:
                    vitesseCible = SHOOTER_VITESSE_POSITION_1;
                    break;
                case 2:
                    vitesseCible = SHOOTER_VITESSE_POSITION_2;
                    break;
                case 3:
                    vitesseCible = SHOOTER_VITESSE_POSITION_3;
                    break;
            }

            shooter.setVelocity(vitesseCible);

            double vitesseActuelle = shooter.getVelocity();
            boolean aLaVitesse = Math.abs(vitesseActuelle - vitesseCible) < SHOOTER_TOLERANCE;

            //telemetry.addData("Shooter", "ACTIF");
            //telemetry.addData("Vitesse Cible", "%.0f ticks/sec", vitesseCible);
            //telemetry.addData("Vitesse Actuelle", "%.0f ticks/sec", vitesseActuelle);
            //telemetry.addData("Erreur", "%.0f ticks/sec", vitesseCible - vitesseActuelle);
            //telemetry.addData("Prêt à tirer", aLaVitesse ? "OUI ✓" : "NON...");
        } else {
            shooter.setVelocity(0);
            //telemetry.addData("Shooter", "ARRÊTÉ");
        }
    }

    // ===================================
    // GESTION ANGLE SHOOTER
    // ===================================
    private void gestionAngleShooter() {
        if (tempsDebounceAngleShooter.milliseconds() > 200) {
            if (gamepad1.circle) {
                positionAngleShooter++;
                if (positionAngleShooter > 3) {
                    positionAngleShooter = 1;
                }
                tempsDebounceAngleShooter.reset();
            }
        }

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

    private static final double VITESSE_TURRET = 0.5;

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

        shooter.setVelocity(0);
        intakeMoteur.setPower(0);
        montageDroit.setPower(0);
        montageGauche.setPower(0);

        servoMoteurRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoRamassageDroit.setPower(0);
        servoRamassageGauche.setPower(0);
        servoAngleShooter.setPosition(0);
    }

    private void gestionMontage() {
        if (tempsDebonceMontage.milliseconds() > 400) {
            if (gamepad1.x) {
                montageActif = !montageActif;
                tempsDebonceMontage.reset();
            }
        }

        if (montageActif) {

            servoRamassageGauche.setPower(1.0);
            servoRamassageDroit.setPower(-1.0);
            servoMoteurRamassageBalle.setPower(0.5);
            montageGauche.setPower(1.0); // normalemtnt négatif mais j'ai changer sur l'initialisation
            montageDroit.setPower(1.0);
        } else {
            servoRamassageGauche.setPower(0.0);
            servoRamassageDroit.setPower(0.0);
            intakeMoteur.setPower(0.0);
            montageGauche.setPower(0.0);
            montageDroit.setPower(0.0);
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
            intakeMoteur.setPower(0.7);
            servoMoteurRamassageBalle.setPower(-1.0);
        } else {
            intakeMoteur.setPower(0.0);
            servoMoteurRamassageBalle.setPower(0.0);
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

        //telemetry.addData("Vitesse Deplacement", "%.0f%%", vitesseDeplacement * 100);
    }

    private void initialisationDuRobot() {
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit"); // 1
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche"); // 2
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit"); // 3
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche"); // 4
        shooter = hardwareMap.get(DcMotorEx.class, "shooter"); // position 1
        intakeMoteur = hardwareMap.get(DcMotor.class, "intake"); // position 2
        montageGauche = hardwareMap.get(DcMotor.class, "montageG"); // position 4
        montageDroit = hardwareMap.get(DcMotor.class, "montageD"); // position 3


        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage"); // position 0
        servoRamassageDroit = hardwareMap.get(CRServo.class, "ramassageD"); // position 5
        servoRamassageGauche = hardwareMap.get(CRServo.class, "ramassageG"); // position 1
        servoTurret = hardwareMap.get(CRServo.class, "turret"); // position 4
        servoAngleShooter = hardwareMap.get(Servo.class, "angleShooter"); // position 3
        // Lumiere Gobilda position = 2

        servoAngleShooter.setPosition(ANGLE_POSITION_1);

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intakeMoteur.setDirection(DcMotor.Direction.REVERSE);
        montageGauche.setDirection(DcMotor.Direction.REVERSE);
        montageDroit.setDirection(DcMotorSimple.Direction.FORWARD);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intakeMoteur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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