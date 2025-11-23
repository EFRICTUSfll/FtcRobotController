package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Tag Follower", group = "Test")
public class TagFollowerOpMode extends LinearOpMode {

    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private WebcamName webcam;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Constants
    private static final int TARGET_TAG_ID = 1;  // Change selon ton Tag
    private static final double TARGET_DISTANCE = 0.20; // 20cm en mètres
    private static final double APPROACH_SPEED = 0.3;   // Vitesse d'approche

    // PID constants
    private static final double ROTATE_KP = 0.01;
    private static final double DISTANCE_KP = 0.5;

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        telemetry.addData("Status", "Prêt - Attend le Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            followTag();
            updateTelemetry();
            sleep(20);
        }
    }

    private void initHardware() {
        // Initialisation des moteurs
        frontLeft = hardwareMap.get(DcMotor.class, "avantGauche");
        frontRight = hardwareMap.get(DcMotor.class, "avantDroit");
        backLeft = hardwareMap.get(DcMotor.class, "dosGauche");
        backRight = hardwareMap.get(DcMotor.class, "dosDroit");

        // Configuration des directions (à ajuster selon ton robot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Freinage pour plus de précision
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcam);
        builder.addProcessor(aprilTag);
        builder.setCameraResolution(new Size(640, 480));

        visionPortal = builder.build();
    }

    private void followTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection target = null;

            // Chercher le Tag cible
            for (AprilTagDetection detection : detections) {
                if (detection.id == TARGET_TAG_ID) {
                    target = detection;
                    break;
                }
            }

            if (target != null) {
                // 1. CALCULER LA ROTATION (garder le Tag centré)
                double tagCenterX = target.center.x;
                double cameraCenterX = 320; // Centre de l'image
                double rotationError = tagCenterX - cameraCenterX;
                double rotatePower = rotationError * ROTATE_KP;

                // 2. CALCULER LA DISTANCE (s'approcher à 20cm)
                double currentDistance = target.ftcPose.range;
                double distanceError = currentDistance - TARGET_DISTANCE;
                double forwardPower = distanceError * DISTANCE_KP;

                // Limiter les puissances
                rotatePower = Range.clip(rotatePower, -0.5, 0.5);
                forwardPower = Range.clip(forwardPower, -0.5, 0.5);

                // 3. APPLIQUER AUX MOTEURS
                driveRobot(forwardPower, rotatePower);

                telemetry.addData("Action", "SUIVI - Distance: %.2fm", currentDistance);

            } else {
                // Tag cible non trouvé
                stopRobot();
                telemetry.addData("Action", "TAG CIBLE NON TROUVE");
            }
        } else {
            // Aucun Tag détecté
            stopRobot();
            telemetry.addData("Action", "AUCUN TAG DETECTE - ARRET");
        }
    }

    private void driveRobot(double forward, double rotate) {
        // avancer + tourner
        double frontLeftPower = forward + rotate;
        double frontRightPower = forward - rotate;
        double backLeftPower = forward + rotate;
        double backRightPower = forward - rotate;

        // les puissances ne doivent pas dépasser 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void updateTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addData("Tags détectés", detections != null ? detections.size() : 0);

        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                telemetry.addData("Tag " + detection.id,
                        "Dist: %.2fm, X: %.0f", detection.ftcPose.range, detection.center.x);
            }
        }

        telemetry.addData("Puissance Moteurs",
                "FL: %.2f, FR: %.2f", frontLeft.getPower(), frontRight.getPower());

        telemetry.update();
    }
}