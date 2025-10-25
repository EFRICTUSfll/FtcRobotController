package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class MyFIRSTJavaOpMode extends LinearOpMode {
    public static final double VITESSE = 0.25;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);


        while (opModeInInit()) {

        }

        leftDrive.setPower(VITESSE);
        sleep(5000);
        leftDrive.setPower(0);


    }
}