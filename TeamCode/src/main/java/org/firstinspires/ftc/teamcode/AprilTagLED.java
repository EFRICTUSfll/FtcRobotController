package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "AprilTag LED")
public class AprilTagLED extends OpMode {

    private Servo light = null;
    private Limelight3A limelight = null;


    // Positions couleurs REV Blinkin
    private static final double COLOR_NO_TAG   = .500; // Vert = tag détecté
    private static final double COLOR_TAG_SEEN = .277; // Rouge  = aucun tag détecté

    @Override
    public void init() {
        // Initialisation du servo LED
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(0.388); // Couleur d'init Jaune

        // Initialisation de la Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);  // Pipeline 0 = AprilTag (à configurer dans l'interface web)
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Récupère le dernier résultat de la Limelight
        LLResult result = limelight.getLatestResult();

        boolean aprilTagDetected = false;

        if (result != null && result.isValid()) {
            // Récupère la liste des AprilTags détectés
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                aprilTagDetected = true;

                // Infos sur le premier tag détecté
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                telemetry.addData("Tag ID", tag.getFiducialId());
                telemetry.addData("TX", tag.getTargetXDegrees());
                telemetry.addData("TY", tag.getTargetYDegrees());
            }
        }

        // Change la LED selon la détection
        if (aprilTagDetected) {
            light.setPosition(COLOR_NO_TAG); // Vert
        } else {
            light.setPosition(COLOR_TAG_SEEN);   // Rouge
        }

        telemetry.addData("AprilTag détecté", aprilTagDetected);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        light.setPosition(.388);
    }
}