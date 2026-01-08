package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class MaxRobot extends LinearOpMode {

    // =========================
    // DRIVE MOTORS
    // =========================
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // =========================
    // ARM MOTORS (ELEVATION)
    // =========================
    DcMotor leftArm;
    DcMotor rightArm;

    // =========================
    // INTAKE SERVO
    // =========================
    CRServo ramassage;

    // =========================
    // IMU
    // =========================
    IMU imu;

    // =========================
    // GLOBAL SPEED
    // =========================
    private double vitesseDeplacement = 0.7;

    // =========================
    // INTAKE TOGGLE
    // =========================
    private boolean ramassageActif = false;
    private boolean l1Precedent = false;

    @Override
    public void runOpMode() {

        initialisationDuRobot();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // SPEED CONTROL (D-PAD)
            // =========================
            gestionVitesse();

            // =========================
            // JOYSTICKS
            // =========================
            // Right joystick: mecanum translation
            double forward = -gamepad1.right_stick_y;
            double right   =  gamepad1.right_stick_x;

            // Left joystick: rotation
            double rotate  =  gamepad1.left_stick_x;

            // =========================
            // DRIVE
            // =========================
            drive(forward, right, rotate);

            // =========================
            // INTAKE SERVO (L1 TOGGLE)
            // =========================
            boolean l1Actuel = gamepad1.left_bumper;

            if (l1Actuel && !l1Precedent) {
                ramassageActif = !ramassageActif;
            }

            ramassage.setPower(ramassageActif ? 1.0 : 0.0);
            l1Precedent = l1Actuel;

            // =========================
            // ARM ELEVATION (L2 / R2)
            // =========================
            double armUp   = gamepad1.right_trigger; // R2
            double armDown = gamepad1.left_trigger;  // L2

            double armPower = armUp - armDown;
            armPower *= 0.6; // precision & stability

            leftArm.setPower(armPower);
            rightArm.setPower(armPower);

            telemetry.update();
        }

        // =========================
        // STOP ALL
        // =========================
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        leftArm.setPower(0);
        rightArm.setPower(0);
        ramassage.setPower(0);
    }

    // ==================================================
    // SPEED MANAGEMENT
    // ==================================================
    private void gestionVitesse() {

        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            vitesseDeplacement += 0.1;
            if (vitesseDeplacement > 1.0) vitesseDeplacement = 1.0;
            sleep(200);
        }

        if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            vitesseDeplacement -= 0.1;
            if (vitesseDeplacement < 0.1) vitesseDeplacement = 0.1;
            sleep(200);
        }
    }

    // ==================================================
    // HARDWARE INIT
    // ==================================================
    private void initialisationDuRobot() {

        frontRightDrive = hardwareMap.get(DcMotor.class, "avantDroit");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "avantGauche");
        backRightDrive  = hardwareMap.get(DcMotor.class, "dosDroit");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "dosGauche");

        leftArm  = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");

        ramassage = hardwareMap.get(CRServo.class, "ramassage");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE = ultra stable hold
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
    }

    // ==================================================
    // MECANUM DRIVE
    // ==================================================
    public void drive(double forward, double right, double rotate) {

        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower  /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower   /= maxPower;
            backRightPower  /= maxPower;
        }

        frontLeftDrive.setPower(vitesseDeplacement * frontLeftPower);
        frontRightDrive.setPower(vitesseDeplacement * frontRightPower);
        backLeftDrive.setPower(vitesseDeplacement * backLeftPower);
        backRightDrive.setPower(vitesseDeplacement * backRightPower);
    }
}
