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
    // Moteur de déplacement
    // =========================
    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;

    CRServo servoMoteurRamassageBalle;

    IMU imu;

    private double vitesseDeplacement = 0.7;

    private boolean estRamassageActif = false;

    private boolean etatPrecedentL1 = false;

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        waitForStart();

        while (opModeIsActive()) {
            gestionVitesseDeplacement();
            deplacement();
            gestionRamassage();

            telemetry.update();
        }

        arretMoteurs();
    }

    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
    }

    private void gestionRamassage() {
        boolean l1Actuel = gamepad1.left_bumper;

        if (l1Actuel && !etatPrecedentL1) {
            estRamassageActif = !estRamassageActif;
        }

        servoMoteurRamassageBalle.setPower(estRamassageActif ? 1.0 : 0.0);
        etatPrecedentL1 = l1Actuel;
    }

    private void gestionVitesseDeplacement() {
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            vitesseDeplacement += 0.1;
            if (vitesseDeplacement > 1.0) vitesseDeplacement = 1.0;
//            sleep(200);
        }

        if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            vitesseDeplacement -= 0.1;
            if (vitesseDeplacement < 0.1) vitesseDeplacement = 0.1;
  //          sleep(200);
        }
    }

    private void initialisationDuRobot() {
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche");

        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, // A FAIRE
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
    }

    public void deplacement() {
        // joystick Droit:  rotation mecanum
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;

        // joystick Gauche : rotation
        double rotate  =  gamepad1.left_stick_x;

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

        moteurAvantGauche.setPower(vitesseDeplacement * frontLeftPower);
        moteurAvantDroit.setPower(vitesseDeplacement * frontRightPower);
        moteurArriereGauche.setPower(vitesseDeplacement * backLeftPower);
        moteurArriereDroit.setPower(vitesseDeplacement * backRightPower);
    }
}
