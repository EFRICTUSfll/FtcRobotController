package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily;

import java.util.List;

//@TeleOp(name = "Test Vision Simple", group = "Test")
public class SimpleVisionTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        initVision();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine("=== TAG DETECTE ===");
                    telemetry.addData("ID", detection.id);
                    telemetry.addData("Distance", "%.2f mètres", detection.ftcPose.range);
                    telemetry.addData("Position X", "%.0f pixels", detection.center.x);
                    telemetry.addData("Position Y", "%.0f pixels", detection.center.y);

                    // Indication gauche/droite
                    if (detection.center.x < 300) {
                        telemetry.addData("Direction", "⬅️ GAUCHE");
                    } else if (detection.center.x > 340) {
                        telemetry.addData("Direction", "➡️ DROITE");
                    } else {
                        telemetry.addData("Direction", "🎯 CENTRE");
                    }
                }
            } else {
                telemetry.addData("Status", "❌ AUCUN TAG VU");
            }

            telemetry.update();
            sleep(100);
        }
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();
    }
}