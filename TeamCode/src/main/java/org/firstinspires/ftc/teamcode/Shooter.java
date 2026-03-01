package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Test - Shooter")
public class Shooter extends LinearOpMode {

    DcMotor shooter;
    IMU imu;


    // ── Toggle shooter ─────
    private boolean shooterActif = false; // état ON/OFF du shooter
    private boolean rightBumperPrecedent = false; // état du bouton au tour précédent

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        waitForStart();

        while (opModeIsActive()) {
            deplacement();
            telemetry.update();
        }

        arretMoteurs();
    }

    private void arretMoteurs() {
        shooter.setPower(0);
    }

    private void initialisationDuRobot() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    public void deplacement() {

        // ── Détection du front montant du right bumper ──────────
        // On ne bascule qu'au MOMENT où on appuie (pas tant qu'on maintient)
        boolean rightBumperActuel = gamepad1.right_bumper;

        if (rightBumperActuel && !rightBumperPrecedent) {
            // Le bouton vient d'être enfoncé → inverser l'état
            shooterActif = !shooterActif;
        }

        rightBumperPrecedent = rightBumperActuel;

        // ── Appliquer la puissance selon l'état ─────────────────
        if (shooterActif) {
            shooter.setPower(-1);
        } else {
            shooter.setPower(0);
        }

        // ── Télémétrie ──────────────────────────────────────────
        telemetry.addData("Shooter", shooterActif ? "✅ ON" : "⛔ OFF");
        telemetry.addData("Bouton RB", rightBumperActuel ? "Appuyé" : "Relâché");
    }
}