import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

// ===== IMPORTS PEDRO PATHING =====
// ⚠️ Assure-toi que Pedro Pathing est installé dans ton projet !
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonome Pedro Pathing", group = "Autonome")
public class AutonomePedroPathing extends LinearOpMode {

    // =========================
    // HARDWARE DU ROBOT (tous les moteurs et servos)
    // =========================
    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;
    DcMotor shooter;
    DcMotor montageMoteur;

    CRServo servoRamassageBalle;
    CRServo servoMoteurRamassageBalle;
    CRServo servoMontageGauche;
    CRServo servoTurret;           // Rotation 360° du shooter
    Servo servoAngleShooter;       // Angle du shooter (3 positions)

    IMU imu;
    GoBildaPinpointDriver odo;     // goBILDA Odometry Computer

    // =========================
    // PEDRO PATHING
    // =========================
    private Follower follower;

    // =========================
    // CONSTANTES
    // =========================
    // Offsets odométrie (ajuste selon ton robot)
    private static final double POD_X_OFFSET_MM = 84.0;
    private static final double POD_Y_OFFSET_MM = -58.0;

    // Positions servo angle shooter
    private static final double ANGLE_SHOOTER_BAS = 0.0;
    private static final double ANGLE_SHOOTER_MILIEU = 0.11;
    private static final double ANGLE_SHOOTER_HAUT = 0.22;

    @Override
    public void runOpMode() {
        // Initialisation complète du robot
        initialisationRobot();

        // ===== INITIALISATION PEDRO PATHING =====
        // Le Follower utilise l'odométrie pour suivre les trajectoires
        follower = new Follower(hardwareMap);

        // ⚠️ Configure la position de départ du robot (en pouces sur le terrain)
        // Exemple : robot démarre à x=56, y=8, face à 90°
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        // ===== CRÉATION DES CHEMINS =====
        Paths paths = new Paths(follower);

        // Affichage
        telemetry.addData("✅ Status", "Robot prêt - Pedro Pathing");
        telemetry.addData("📍 Position départ", "x=56, y=8, heading=90°");
        telemetry.addData("⚠️ VÉRIFIE", "Pods touchent le sol !");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // ===================================
            // EXÉCUTION DES CHEMINS
            // ===================================

            // PATH 1 : Ligne droite
            telemetry.addData("📍", "Path 1 en cours...");
            telemetry.update();
            follower.followPath(paths.Path1);

            // Pendant que le robot suit le path, on peut faire d'autres actions
            while (follower.isBusy() && opModeIsActive()) {
                follower.update();  // ⚠️ IMPORTANT : met à jour Pedro Pathing
                telemetry.addData("Path", "1 - En cours");
                telemetry.addData("Position", follower.getPose().toString());
                telemetry.update();

            }

            // PATH 2 : Courbe avec shooter actif
            telemetry.addData("📍", "Path 2 en cours...");
            telemetry.update();
            follower.followPath(paths.Path2);

            // Exemple : activer le shooter pendant le trajet
            activerShooter(true);
            setAngleShooter(2);  // Position haute

            while (follower.isBusy() && opModeIsActive()) {
                follower.update();
                telemetry.addData("Path", "2 - En cours + Shooter ON");
                telemetry.update();
            }

            // Désactiver le shooter
            activerShooter(false);

            // PATH 3 : Dernière courbe avec ramassage
            telemetry.addData("📍", "Path 3 en cours...");
            telemetry.update();
            follower.followPath(paths.Path3);

            // Exemple : activer le ramassage
            activerRamassage(true);

            while (follower.isBusy() && opModeIsActive()) {
                follower.update();
                telemetry.addData("Path", "3 - En cours + Ramassage ON");
                telemetry.update();
            }

            activerRamassage(false);

            // ===================================
            // FIN DE L'AUTONOME
            // ===================================
            telemetry.addData("✅", "Autonome terminé !");
            telemetry.addData("Position finale", follower.getPose().toString());
            telemetry.update();

            arretTout();
        }
    }

    /**
     * INITIALISATION COMPLÈTE DU ROBOT
     */
/*
    private void initialisationRobot() {
        // Moteurs de déplacement
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

        // Autres moteurs
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        montageMoteur = hardwareMap.get(DcMotor.class, "montage");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        montageMoteur.setDirection(DcMotor.Direction.REVERSE);

        // Servos
        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageBalle = hardwareMap.get(CRServo.class, "ramassage2");
        servoTurret = hardwareMap.get(CRServo.class, "rotationShooter");
        servoMontageGauche = hardwareMap.get(CRServo.class, "montageGauche");
        servoAngleShooter = hardwareMap.get(Servo.class, "angleShooter");

        // Position initiale angle shooter : BAS
        servoAngleShooter.setPosition(ANGLE_SHOOTER_BAS);

        // goBILDA Odometry Computer
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odoComputer");
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
            odo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            odo.resetPosAndIMU();
            telemetry.addData("✅ Odometry", "Configuré");
        } catch (Exception e) {
            telemetry.addData("❌ Odometry", "Non trouvé !");
        }

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        telemetry.addData("✅ Robot", "Initialisé");
        telemetry.update();
    }

    // ===================================
    // FONCTIONS DE CONTRÔLE DES MÉCANISMES
    // ===================================

    /**
     * Active ou désactive le shooter
     */
/*
    private void activerShooter(boolean actif) {
        if (actif) {
            shooter.setPower(-0.7);
            telemetry.addData("🎯 Shooter", "ON");
        } else {
            shooter.setPower(0);
            telemetry.addData("🎯 Shooter", "OFF");
        }
    }

    /**
     * Active ou désactive le ramassage
     */

/*
    private void activerRamassage(boolean actif) {
        if (actif) {
            servoMoteurRamassageBalle.setPower(-1.0);
            servoRamassageBalle.setPower(1.0);
            telemetry.addData("🔄 Ramassage", "ON");
        } else {
            servoMoteurRamassageBalle.setPower(0);
            servoRamassageBalle.setPower(0);
            telemetry.addData("🔄 Ramassage", "OFF");
        }
    }

    /**
     * Active ou désactive le montage
     */

/*
    private void activerMontage(boolean actif) {
        if (actif) {
            servoMontageGauche.setPower(-0.7);
            montageMoteur.setPower(-0.7);
            telemetry.addData("⬆️ Montage", "ON");
        } else {
            servoMontageGauche.setPower(0);
            montageMoteur.setPower(0);
            telemetry.addData("⬆️ Montage", "OFF");
        }
    }

    /**
     * Rotation du turret (shooter)
     * @param vitesse -1.0 (gauche) à +1.0 (droite), 0 = arrêt
     */

/*
    private void tournerTurret(double vitesse) {
        servoTurret.setPower(vitesse * 0.4);  // 0.4 = vitesse max
    }

    /**
     * Change l'angle du shooter
     * @param position 0=bas, 1=milieu, 2=haut
     */

/*
    private void setAngleShooter(int position) {
        switch (position) {
            case 0:
                servoAngleShooter.setPosition(ANGLE_SHOOTER_BAS);
                telemetry.addData("📐 Angle Shooter", "BAS");
                break;
            case 1:
                servoAngleShooter.setPosition(ANGLE_SHOOTER_MILIEU);
                telemetry.addData("📐 Angle Shooter", "MILIEU");
                break;
            case 2:
                servoAngleShooter.setPosition(ANGLE_SHOOTER_HAUT);
                telemetry.addData("📐 Angle Shooter", "HAUT");
                break;
        }
    }

    /**
     * Arrête tous les moteurs et servos
     */
/*
    private void arretTout() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
        shooter.setPower(0);
        montageMoteur.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoMontageGauche.setPower(0);
    }

    // ===================================
    // CLASSE PATHS (copie tes chemins ici)
    // ===================================
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            // ===== PATH 1 : Ligne droite =====
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000),
                            new Pose(46.010, 59.635)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            // ===== PATH 2 : Courbe avec heading fixe =====
            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(46.010, 59.635),
                            new Pose(66.697, 79.921),
                            new Pose(47.760, 96.244)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(134))
                    .build();

            // ===== PATH 3 : Courbe avec heading tangent =====
            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(47.760, 96.244),
                            new Pose(67.000, 82.900),
                            new Pose(46.500, 84.200)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
*/