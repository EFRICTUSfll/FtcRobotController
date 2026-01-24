package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class MaxRobot extends LinearOpMode {

    // =========================
    // Moteur de déplacement
    // =========================
    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;

    CRServo servoMoteurRamassageBalle;

    CRServo servoMoteurShooter;

    IMU imu;

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    private VisionPortal visionPortal;

    private double vitesseDeplacement = 0.7;

    private boolean estRamassageActif = false;

    private AprilTagProcessor aprilTag;


    @Override
    public void runOpMode() {
        initialisationDuRobot();
        waitForStart();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacement();
            gestionRamassage();
            rotationShooter();
            telemetry.update();
        }

        arretMoteurs();
    }

    private void rotationShooter() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one .
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (detection.metadata.name.contains("BlueTarget")) {
                    telemetry.addLine(String.format("Centre %6.1f", detection.center.x));
                    // Position de l'AprilTag dans l'image
                    if (detection.center.x > 340) {
                        telemetry.addLine(String.format("Tourner Droite"));
                        double rotation = 0.1;
                        tournerShooter(rotation);
                    } else if (detection.center.x < 300) {
                        telemetry.addLine(String.format("Tourner Gauche"));
                        double rotation = -0.1;
                        tournerShooter(rotation);
                    } else {
                        telemetry.addLine(String.format("Attendre"));
                        double rotation = 0;
                        tournerShooter(rotation);
                    }
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop
        if (currentDetections.size()==0){
            telemetry.addLine(String.format("\n==== Rien trouvé"));
            double rotation = 0.1;
            tournerShooter(rotation);
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

    private void tournerShooter(double rotation) {
        servoMoteurShooter.setPower(rotation);
        telemetry.addLine(String.format("%.2f, Rotation Vitesse", rotation));
    }

    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoMoteurShooter.setPower(0);
    }

    private void gestionRamassage() {
        boolean valeurL1 = gamepad1.left_bumper;

        if (valeurL1) {
            estRamassageActif = !estRamassageActif;
        }

        if (estRamassageActif) {
            servoMoteurRamassageBalle.setPower(-1.0);
        } else {
            servoMoteurRamassageBalle.setPower(0.0);
        }
        sleep(200);
    }

    private void gestionVitesseDeplacement() {
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            vitesseDeplacement += 0.1;
            if (vitesseDeplacement > 1.0) vitesseDeplacement = 1.0;
//            sleep(200);
        }

        if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            vitesseDeplacement -= 0.1;
            if (vitesseDeplacement < 0.1) vitesseDeplacement = 0.1;
            //          sleep(200);
        }
    }

    private void initialisationDuRobot() {
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche");

        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoMoteurShooter = hardwareMap.get(CRServo.class, "rotationShooter");

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, // A FAIRE
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();


        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void deplacement() {
        // joystick Droit:  rotation mecanum
        double forward = -gamepad1.right_stick_y;
        double right = gamepad1.right_stick_x;

        // joystick Gauche : rotation
        double rotate = gamepad1.left_stick_x;

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        moteurAvantGauche.setPower(vitesseDeplacement * frontLeftPower);
        moteurAvantDroit.setPower(vitesseDeplacement * frontRightPower);
        moteurArriereGauche.setPower(vitesseDeplacement * backLeftPower);
        moteurArriereDroit.setPower(vitesseDeplacement * backRightPower);

        telemetry.addData("Moteurs", "AV-G: %.2f, AV-D: %.2f, AR-G: %.2f, AR-D: %.2f",
                frontLeftPower, frontRightPower,
                backLeftPower, backRightPower);

        telemetry.addData("Variable", "Forward: %.2f, Right: %.2f, Rotate: %.2f",
                forward, right,
                rotate);

        //sleep(200);
    }
}
