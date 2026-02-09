package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Autonome goBILDA Odometry", group = "Autonome")
public class MaxAutonome extends LinearOpMode {

    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;
    Servo servoAngleShooter;

    GoBildaPinpointDriver odo;

    IMU imu;

    // =========================
    // Configuration des offsets (distances des pods au centre du robot)
    // =========================
    private static final double POD_X_OFFSET_MM = 84.0;   // Pod X : 84mm depuis le centre
    private static final double POD_Y_OFFSET_MM = -58.0;  // Pod Y : -58mm (négatif = à gauche)

    // Positions du servo angle shooter (0.0 à 1.0)
    private static final double ANGLE_SHOOTER_BAS = 0.2;      // Position basse
    private static final double ANGLE_SHOOTER_MILIEU = 0.5;   // Position milieu
    private static final double ANGLE_SHOOTER_HAUT = 0.8;     // Position haute

    // Position actuelle (commence en BAS)
    private int positionShooterActuelle = 0;  // 0=bas, 1=milieu, 2=haut

    // Vitesse
    private static final double VITESSE_DEPLACEMENT = 0.5;
    private static final double VITESSE_ROTATION = 0.35;

    // Tolérance
    private static final double TOLERANCE_DISTANCE_CM = 2.0;
    private static final double TOLERANCE_ANGLE_DEGRES = 3.0;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();
    private static final double TIMEOUT_SECONDES = 10.0;

    @Override
    public void runOpMode() {
        initialisationRobot();

        //telemetry.addData("✅ Status", "Robot prêt - goBILDA Odometry");
        //telemetry.addData("📍 Mission", "Avancer 1m puis tourner 90° droite");
        //telemetry.addData("🎯 Angle Shooter", "Position BASSE");
        //telemetry.addData("⚠️ VÉRIFIE", "Pods touchent le sol !");
        //telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // ÉTAPE 1 : Avancer de 100 cm
            avancerDistance(100);

            sleep(500);

            avancerDistance(100);

            // ÉTAPE 2 : Tourner à droite de 90°
            //tournerAngle(90);

            sleep(500);

            // Position finale
            Pose2D posFinal = odo.getPosition();
            telemetry.addData("✅ Status", "Autonome terminé !");
            telemetry.addData("📍 Position finale", "X: %.1f cm, Y: %.1f cm",
                    posFinal.getX(DistanceUnit.CM),
                    posFinal.getY(DistanceUnit.CM));
            telemetry.addData("🧭 Angle final", "%.1f°", posFinal.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            arretMoteurs();
        }
    }

    private void initialisationRobot() {
        // Moteurs
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche");

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);

        moteurAvantGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurAvantDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo angle shooter
        servoAngleShooter = hardwareMap.get(Servo.class, "angleShooter");
        servoAngleShooter.setPosition(ANGLE_SHOOTER_BAS);  // Démarre en position BASSE

        // goBILDA ODOMETRY COMPUTER
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odoComputer");

            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);

            odo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            odo.resetPosAndIMU();

            telemetry.addData("✅ Odometry Computer", "Détecté");
            telemetry.addData("Pod X Offset", "%.1f mm", POD_X_OFFSET_MM);
            telemetry.addData("Pod Y Offset", "%.1f mm", POD_Y_OFFSET_MM);

        } catch (Exception e) {
            telemetry.addData("❌ ERREUR", "Odometry Computer non trouvé !");
            telemetry.addData("Vérifie", "1. Cable I2C branché");
            telemetry.addData("", "2. Nom = 'odoComputer' dans config");
            telemetry.update();
            sleep(10000);
        }

        // IMU du Control Hub (optionnel)
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();

        telemetry.addData("✅ Robot", "Initialisé");
        telemetry.addData("✅ Servo Shooter", "Position BASSE");
        telemetry.update();
    }

    private void avancerDistance(double distanceCible_cm) {
        odo.resetPosAndIMU();

        telemetry.addData("🎯 Distance cible", "%.1f cm", distanceCible_cm);
        telemetry.update();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TIMEOUT_SECONDES) {
            odo.update();

            Pose2D pos = odo.getPosition();
            double distanceParcourue = pos.getX(DistanceUnit.CM);
            double deriveY = pos.getY(DistanceUnit.CM);
            double distanceRestante = distanceCible_cm - distanceParcourue;

            if (Math.abs(distanceRestante) < TOLERANCE_DISTANCE_CM) {
                break;
            }

            double vitesse = calculerVitesseRampe(distanceRestante, distanceCible_cm);
            double correction = deriveY * 0.02;

            moteurAvantGauche.setPower(vitesse - correction);
            moteurAvantDroit.setPower(vitesse + correction);
            moteurArriereGauche.setPower(vitesse - correction);
            moteurArriereDroit.setPower(vitesse + correction);

            telemetry.addData("📏 Distance parcourue", "%.1f cm", distanceParcourue);
            telemetry.addData("📏 Restante", "%.1f cm", distanceRestante);
            telemetry.addData("📍 Position", "X: %.1f, Y: %.1f",
                    pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM));
            telemetry.addData("🚀 Vitesse", "%.2f", vitesse);
            telemetry.update();

            sleep(10);
        }

        arretMoteurs();
        sleep(200);
    }

    private void tournerAngle(double angleCible_deg) {
        odo.resetPosAndIMU();
        sleep(100);

        telemetry.addData("🎯 Rotation cible", "%.1f°", angleCible_deg);
        telemetry.update();

        double anglePrecedent = 0;
        int compteurOscillation = 0;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TIMEOUT_SECONDES) {
            odo.update();

            Pose2D pos = odo.getPosition();
            double angleActuel = pos.getHeading(AngleUnit.DEGREES);
            double erreurAngle = angleCible_deg - angleActuel;

            while (erreurAngle > 180) erreurAngle -= 360;
            while (erreurAngle < -180) erreurAngle += 360;

            if (Math.abs(erreurAngle) < TOLERANCE_ANGLE_DEGRES) {
                telemetry.addData("✅", "Angle atteint !");
                telemetry.update();
                break;
            }

            if ((anglePrecedent > 0 && angleActuel < 0) || (anglePrecedent < 0 && angleActuel > 0)) {
                compteurOscillation++;
                if (compteurOscillation > 3) {
                    break;
                }
            }
            anglePrecedent = angleActuel;

            double vitesse = calculerVitesseRotation(erreurAngle);

            if (erreurAngle > 0) {
                moteurAvantGauche.setPower(vitesse);
                moteurArriereGauche.setPower(vitesse);
                moteurAvantDroit.setPower(-vitesse);
                moteurArriereDroit.setPower(-vitesse);
            } else {
                moteurAvantGauche.setPower(-vitesse);
                moteurArriereGauche.setPower(-vitesse);
                moteurAvantDroit.setPower(vitesse);
                moteurArriereDroit.setPower(vitesse);
            }

            telemetry.addData("🧭 Angle actuel", "%.1f°", angleActuel);
            telemetry.addData("🎯 Cible", "%.1f°", angleCible_deg);
            telemetry.addData("📐 Erreur", "%.1f°", erreurAngle);
            telemetry.update();

            sleep(20);
        }

        arretMoteurs();

        odo.update();
        Pose2D posFinal = odo.getPosition();
        double angleFinal = posFinal.getHeading(AngleUnit.DEGREES);
        telemetry.addData("✅ Angle final", "%.1f°", angleFinal);
        telemetry.addData("🎯 Précision", "%.1f°", Math.abs(angleCible_deg - angleFinal));
        telemetry.update();

        sleep(200);
    }

    private double calculerVitesseRampe(double distanceRestante, double distanceTotale) {
        double distanceRampe = distanceTotale * 0.3;

        if (Math.abs(distanceRestante) < distanceRampe) {
            double vitesse = VITESSE_DEPLACEMENT * (Math.abs(distanceRestante) / distanceRampe);
            return Math.max(vitesse, 0.15);
        }

        return VITESSE_DEPLACEMENT;
    }

    private double calculerVitesseRotation(double erreurAngle) {
        double angleRampe = 20.0;

        if (Math.abs(erreurAngle) < angleRampe) {
            double vitesse = VITESSE_ROTATION * (Math.abs(erreurAngle) / angleRampe);
            return Math.max(vitesse, 0.15);
        }

        return VITESSE_ROTATION;
    }

    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
    }

    /**
     * Change l'angle du shooter (3 positions : bas, milieu, haut)
     * Utilise cette fonction dans ton autonome quand tu veux changer l'angle
     */
    private void changerAngleShooter() {
        // Cycle entre les 3 positions : bas → milieu → haut → bas...
        positionShooterActuelle++;
        if (positionShooterActuelle > 2) {
            positionShooterActuelle = 0;
        }

        // Applique la position correspondante
        switch (positionShooterActuelle) {
            case 0:  // Position BASSE
                servoAngleShooter.setPosition(ANGLE_SHOOTER_BAS);
                telemetry.addData("🎯 Angle Shooter", "BAS");
                break;
            case 1:  // Position MILIEU
                servoAngleShooter.setPosition(ANGLE_SHOOTER_MILIEU);
                telemetry.addData("🎯 Angle Shooter", "MILIEU");
                break;
            case 2:  // Position HAUTE
                servoAngleShooter.setPosition(ANGLE_SHOOTER_HAUT);
                telemetry.addData("🎯 Angle Shooter", "HAUT");
                break;
        }
        telemetry.update();
    }

    /**
     * Met directement le shooter à une position spécifique (sans cycle)
     * Utile pour l'autonome
     */
    private void setAngleShooter(int position) {
        positionShooterActuelle = position;

        switch (position) {
            case 0:  // BAS
                servoAngleShooter.setPosition(ANGLE_SHOOTER_BAS);
                telemetry.addData("🎯 Angle Shooter", "BAS");
                break;
            case 1:  // MILIEU
                servoAngleShooter.setPosition(ANGLE_SHOOTER_MILIEU);
                telemetry.addData("🎯 Angle Shooter", "MILIEU");
                break;
            case 2:  // HAUT
                servoAngleShooter.setPosition(ANGLE_SHOOTER_HAUT);
                telemetry.addData("🎯 Angle Shooter", "HAUT");
                break;
        }
        telemetry.update();
    }
}