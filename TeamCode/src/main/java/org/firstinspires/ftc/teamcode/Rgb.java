package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Rgb extends OpMode{

    public Servo light = null;
    @Override
    public void init(){
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(.388);
    }

    @Override
    public void loop(){
        light.setPosition(.500);
        light.setPosition(.160);
    }
}