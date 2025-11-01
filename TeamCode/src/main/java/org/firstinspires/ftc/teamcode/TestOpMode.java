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

        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0f) {
                int speed = (int) (gamepad1.right_trigger * 100);
                telemetry.addLine("Speed : " + speed);
                telemetry.update();
                move(-speed, speed, -speed, speed);
            } else if (gamepad1.left_trigger > 0f) {
                int speed = (int) (gamepad1.left_trigger * 100);
                telemetry.addLine("Speed : " + speed);
                telemetry.update();
                move(speed, -speed, speed, -speed);
            } else {
                move(0, 0, 0, 0);
            }
        }
    }

    public void move(int frontLeft, int frontRight, int backLeft, int backRight) {
        frontRightDrive.setPower(frontRight);
        frontLeftDrive.setPower(-frontLeft);
        backLeftDrive.setPower(-backLeft);
        backRightDrive.setPower(backRight);
    }
}