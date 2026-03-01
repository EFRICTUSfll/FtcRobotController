 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "Test 2")
public class TestMax extends LinearOpMode {

    CRServo montage1;
    CRServo montage2;
    CRServo montage3;
    CRServo montage4;

    DcMotorEx montageMoteur;
    DcMotorEx moteurMontage;

    IMU imu;

    private boolean estRamassageActif = false;
    private boolean lastLeftBumper = false;
    private boolean montageActif = false;

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        waitForStart();

        while (opModeIsActive()) {
            gestionRamassage();
            gestionMontage();
            telemetry.update();
        }

        arretMoteurs();
    }

    private void arretMoteurs() {
        montage1.setPower(0);
        montage2.setPower(0);
        montage3.setPower(0);
        montage4.setPower(0);
        montageMoteur.setPower(0);
        moteurMontage.setPower(0);
    }

    private void gestionRamassage() {

        boolean bumperActuel = gamepad1.left_bumper;

        if (bumperActuel && !lastLeftBumper) {
            estRamassageActif = !estRamassageActif;
        }
        lastLeftBumper = bumperActuel;

        // Appliquer la puissance
        if (estRamassageActif) {
            montage1.setPower(1.0);
            montage2.setPower(-1.0);
            montage3.setPower(-1.0);
            montage4.setPower(1.0);

            telemetry.addData("Montage", "✅ ACTIF");
        } else {
            montage1.setPower(0.0);
            montage2.setPower(0.0);
            montage3.setPower(0.0);
            montage4.setPower(0.0);

            telemetry.addData("Montage", "⛔ ARRÊTÉ");
        }
    }
    private void gestionMontage() {

        boolean bumperActuel = gamepad1.right_bumper;

        if (bumperActuel && !estRamassageActif) {
            estRamassageActif = !estRamassageActif;
        }
        estRamassageActif = bumperActuel;

        // Appliquer la puissance
        if (estRamassageActif) {
            montageMoteur.setPower(1.0);
            moteurMontage.setPower(1.0);

            telemetry.addData("Montage", "✅ ACTIF");
        } else {
            montageMoteur.setPower(0.0);
            moteurMontage.setPower(0.0);

            telemetry.addData("Montage", "⛔ ARRÊTÉ");
        }
    }



    private void initialisationDuRobot() {

        montageMoteur = hardwareMap.get(DcMotorEx.class, "montage");
        moteurMontage = hardwareMap.get(DcMotorEx.class, "montageM");

        montage1 = hardwareMap.get(CRServo.class, "montage1");
        montage2 = hardwareMap.get(CRServo.class, "montage2");
        montage3 = hardwareMap.get(CRServo.class, "montage3");
        montage4 = hardwareMap.get(CRServo.class, "montage4");

        montageMoteur.setDirection(DcMotorEx.Direction.REVERSE);
        moteurMontage.setDirection(DcMotorEx.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

    }
}
