package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RobotTourneOpMode extends LinearOpMode {

    public static final double VITESSE = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor avantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        DcMotor avantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        DcMotor dosDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        DcMotor dosGauche = hardwareMap.get(DcMotor.class, "dosGauche");

        avantDroit.setDirection(DcMotor.Direction.FORWARD);
        avantGauche.setDirection(DcMotor.Direction.REVERSE);
        dosDroit.setDirection(DcMotor.Direction.FORWARD);
        dosGauche.setDirection(DcMotor.Direction.REVERSE);

        while (opModeInInit()) {

        }

        for (double vitesse = 0; vitesse <= 1; vitesse = vitesse + 0.1) {

            avantDroit.setPower(-vitesse);
            avantGauche.setPower(vitesse);
            dosDroit.setPower(-vitesse);
            dosGauche.setPower(vitesse);
            sleep(1500);
        }

        avantDroit.setPower(0);
        avantGauche.setPower(0);
        dosDroit.setPower(0);
        dosGauche.setPower(0);
    }
}
