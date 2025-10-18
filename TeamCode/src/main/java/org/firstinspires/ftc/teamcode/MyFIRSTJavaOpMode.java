package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class MyFIRSTJavaOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        testDrive = hardwareMap.get(DcMotor.class, "testDrive");

    }
}