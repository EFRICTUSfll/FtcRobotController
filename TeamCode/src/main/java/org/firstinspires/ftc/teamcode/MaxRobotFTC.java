package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "MAX")
public class MaxRobotFTC extends LinearOpMode {
    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;
    DcMotor shooter;
    DcMotor montageMoteur;

    CRServo servoRamassageBalle;
    CRServo servoMoteurRamassageBalle;
    CRServo servoMontageGauche;
    CRServo servoTurret;  // La rotation du shooter
    Servo servoAngleShooter;  // Servo pour régler l'angle du shooter

    IMU imu;

    private double vitesseDeplacement = 0.7;

    private boolean shooterActif = false;
    private boolean estRamassageActif = false;
    private boolean montageActif = false;

    private int positionAngleShooter = 1;  // Position actuelle : 1, 2, ou 3

    // ===================================
    // Augmente la fluidité du robot, car plus de 10K = pas stable
    // ===================================
    private double puissanceAvantGaucheActuelle = 0;
    private double puissanceAvantDroitActuelle = 0;
    private double puissanceArriereGaucheActuelle = 0;
    private double puissanceArriereDroitActuelle = 0;

    // acceleration avec joystick
    private final double TAUX_ACCELERATION = 0.25;  // Réponse rapide au joystick
    private final double TAUX_DECELERATION = 0.35;  // Arrêt rapide quand on relâche
    private final double SEUIL_MORT = 0.05;         // Zone morte du joystick

    private ElapsedTime tempsLoop = new ElapsedTime();
    private ElapsedTime tempsDebounceVitesse = new ElapsedTime();
    private ElapsedTime tempsDebounceRamassage = new ElapsedTime(); // remplace sleep(200)
    private ElapsedTime tempsDebounceShooter = new ElapsedTime();   // remplace sleep(150)
    private ElapsedTime tempsDebonceMontage = new ElapsedTime();    // remplace sleep(150)
    private ElapsedTime tempsDebounceAngleShooter = new ElapsedTime(); // bouton cercle

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        waitForStart();

        tempsLoop.reset();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacementFluide();  // ← FONCTION MODIFIÉE
            gestionRamassage();
            rotationTurret();
            gestionMontage();
            gestionShooter();
            gestionAngleShooter();  // Bouton cercle pour changer l'angle

            telemetry.update();
        }

        arretMoteurs();
    }

    // ===================================
    // Le shooter tourne a 360°
    // ===================================
    private static final double VITESSE_TURRET = 0.4; // Vitesse de rotation du turret (0.0 à 1.0)

    private void rotationTurret() {
        float gachetteGauche = gamepad1.left_trigger;   // Boutton L2
        float gachetteDroit  = gamepad1.right_trigger;  // Boutton R2

        if (gachetteDroit > 0.1) {
            // Tourne a droite en fonction de la force sur la gachette
            servoTurret.setPower(gachetteDroit * VITESSE_TURRET);
        } else if (gachetteGauche > 0.1) {
            // Tourne à gauche
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
        shooter.setPower(0);
        montageMoteur.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoRamassageBalle.setPower(0);
        servoMontageGauche.setPower(0);
    }

    private void gestionMontage() {
        // Debounce : on ne lit le bouton que toutes les 150ms
        if (tempsDebonceMontage.milliseconds() > 150) {
            if (gamepad1.triangle) {
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
        // PAS DE sleep() ICI
    }

    private void gestionRamassage() {
        // Debounce : on ne lit le bouton que toutes les 200ms
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
        // PAS DE sleep() ICI
    }

    private void gestionVitesseDeplacement() {
        // On n'agit que si 300ms se sont écoulées depuis le dernier changement
        // → un seul changement par appui, même si le bouton reste enfoncé
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

    private void gestionShooter() {
        // Debounce : on ne lit le bouton que toutes les 150ms
        if (tempsDebounceShooter.milliseconds() > 150) {
            if (gamepad1.x) {
                shooterActif = !shooterActif;
                tempsDebounceShooter.reset();
            }
        }

        if (shooterActif) {
            shooter.setPower(-0.7);
        } else {
            shooter.setPower(0.0);
        }
        // PAS DE sleep() ICI
    }

    private void gestionAngleShooter() {

        final double POSITION_1 = 0.0;    // 0° → setPosition(0.0)
        final double POSITION_2 = 0.11;   // ~20° → setPosition(0.11) sur un servo 180°
        final double POSITION_3 = 0.22;   // ~40° → setPosition(0.22)

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

        // Appliquer la position au servo
        if (positionAngleShooter == 1) {
            servoAngleShooter.setPosition(POSITION_1);
        } else if (positionAngleShooter == 2) {
            servoAngleShooter.setPosition(POSITION_2);
        } else {  // position 3
            servoAngleShooter.setPosition(POSITION_3);
        }

        telemetry.addData("Angle Shooter", "Position %d/3", positionAngleShooter);
    }

    private void initialisationDuRobot() {
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        montageMoteur = hardwareMap.get(DcMotor.class, "montage");

        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageBalle = hardwareMap.get(CRServo.class, "ramassage2");
        servoTurret = hardwareMap.get(CRServo.class, "rotationShooter");  // 360° Torque
        servoMontageGauche = hardwareMap.get(CRServo.class, "montageGauche");
        servoAngleShooter = hardwareMap.get(Servo.class, "angleShooter");  // Servo angle shooter

        // Position initiale de l'angle shooter : position 1 (tout en bas)
        servoAngleShooter.setPosition(0.0);

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
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        montageMoteur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMPORTANT : Mode BRAKE pour robot lourd (arrêt plus contrôlé)
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
    // NOUVELLE FONCTION DE DÉPLACEMENT FLUIDE
    // ===================================
    public void deplacementFluide() {
        // Lecture des joysticks (identique à votre code original)
        double forward = -gamepad1.right_stick_y;
        double right = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        // Application de la zone morte (évite les micro-mouvements)
        forward = appliquerZoneMorte(forward);
        right = appliquerZoneMorte(right);
        rotate = appliquerZoneMorte(rotate);

        // Calcul des puissances cibles (identique à votre code original)
        double frontLeftPowerCible = forward + right + rotate;
        double frontRightPowerCible = forward - right - rotate;
        double backRightPowerCible = forward + right - rotate;
        double backLeftPowerCible = forward - right + rotate;

        // Normalisation (identique à votre code original)
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

        // ===================================
        // PARTIE NOUVELLE : RAMPE D'ACCÉLÉRATION
        // ===================================
        // Au lieu d'appliquer directement la puissance cible, on fait une transition progressive
        puissanceAvantGaucheActuelle = appliquerRampe(puissanceAvantGaucheActuelle, frontLeftPowerCible);
        puissanceAvantDroitActuelle = appliquerRampe(puissanceAvantDroitActuelle, frontRightPowerCible);
        puissanceArriereGaucheActuelle = appliquerRampe(puissanceArriereGaucheActuelle, backLeftPowerCible);
        puissanceArriereDroitActuelle = appliquerRampe(puissanceArriereDroitActuelle, backRightPowerCible);

        // Application des puissances progressives avec votre multiplicateur de vitesse
        moteurAvantGauche.setPower(vitesseDeplacement * puissanceAvantGaucheActuelle);
        moteurAvantDroit.setPower(vitesseDeplacement * puissanceAvantDroitActuelle);
        moteurArriereGauche.setPower(vitesseDeplacement * puissanceArriereGaucheActuelle);
        moteurArriereDroit.setPower(vitesseDeplacement * puissanceArriereDroitActuelle);

        // Télémétrie améliorée
        telemetry.addData("Vitesse Max", "%.0f%%", vitesseDeplacement * 100);
        telemetry.addData("Moteurs Cible", "AV-G: %.2f, AV-D: %.2f, AR-G: %.2f, AR-D: %.2f",
                frontLeftPowerCible, frontRightPowerCible,
                backLeftPowerCible, backRightPowerCible);
        telemetry.addData("Moteurs Réel", "AV-G: %.2f, AV-D: %.2f, AR-G: %.2f, AR-D: %.2f",
                puissanceAvantGaucheActuelle, puissanceAvantDroitActuelle,
                puissanceArriereGaucheActuelle, puissanceArriereDroitActuelle);
        telemetry.addData("Joysticks", "Forward: %.2f, Right: %.2f, Rotate: %.2f",
                forward, right, rotate);
    }

    /**
     * Applique une rampe entre la valeur actuelle et la cible.
     * - Vers une valeur non-nulle : accélération avec TAUX_ACCELERATION
     * - Vers 0 : décélération avec TAUX_DECELERATION (plus rapide)
     * - Proche de 0 : on coupe carrément
     */
    private double appliquerRampe(double valeurActuelle, double valeurCible) {
        // Si la cible est 0 et on est proche → arrêt net
        if (valeurCible == 0 && Math.abs(valeurActuelle) < 0.05) {
            return 0;
        }

        double difference = valeurCible - valeurActuelle;

        // Choisir le taux selon la direction
        double taux = (valeurCible == 0) ? TAUX_DECELERATION : TAUX_ACCELERATION;

        return valeurActuelle + (difference * taux);
    }

    /**
     * Élimine les petits mouvements involontaires du joystick
     */
    private double appliquerZoneMorte(double valeur) {
        if (Math.abs(valeur) < SEUIL_MORT) {
            return 0;
        }
        return valeur;
    }
}