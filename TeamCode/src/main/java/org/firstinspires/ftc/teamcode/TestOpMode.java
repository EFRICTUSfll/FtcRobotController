package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestOpMode extends LinearOpMode {
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;

    @Override
    public void runOpMode() {
        frontRightDrive = hardwareMap.get(DcMotor.class, "avantDroit");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "avantGauche");
        backRightDrive = hardwareMap.get(DcMotor.class, "dosDroit");
        backLeftDrive = hardwareMap.get(DcMotor.class, "dosGauche");

        // Configuration des directions des moteurs (ajuster selon votre robot)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Lecture des joysticks
            double drive = -gamepad1.left_stick_y;    // Avant/Arrière
            double strafe = gamepad1.left_stick_x;    // Gauche/Droite (latéral)
            double rotate = gamepad1.right_stick_x;   // Rotation

            // Calcul des puissances des moteurs pour le Mecanum
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalisation des puissances pour ne pas dépasser 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Application des puissances aux moteurs
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Télémetrie
            telemetry.addData("Joystick", "Drive: %.2f, Strafe: %.2f, Rotate: %.2f", drive, strafe, rotate);
            telemetry.addData("Moteurs", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }

        // Arrêt des moteurs à la fin
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}