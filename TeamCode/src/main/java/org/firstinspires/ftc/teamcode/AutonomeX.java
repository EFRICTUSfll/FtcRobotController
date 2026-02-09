package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonome Simple Timer", group = "Autonome")
public class AutonomeX extends LinearOpMode {

    DcMotor moteurAvantGauche;
    DcMotor moteurAvantDroit;
    DcMotor moteurArriereGauche;
    DcMotor moteurArriereDroit;

    // Vitesse des moteurs
    private static final double VITESSE = 0.5;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initialisationRobot();

        telemetry.addData("✅ Status", "Robot prêt");
        telemetry.addData("📍 Mission", "Avancer 1m puis reculer 1m");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Avancer pendant 2 secondes (environ 1m à vitesse 0.5)
            reculer(1.0);
            sleep(500);

            // Reculer pendant 2 secondes (environ 1m)
            avancer(1.0);

            arretMoteurs();
        }
    }

    private void initialisationRobot() {
        // Moteurs
        moteurAvantDroit = hardwareMap.get(DcMotor.class, "avantDroit");
        moteurAvantGauche = hardwareMap.get(DcMotor.class, "avantGauche");
        moteurArriereDroit = hardwareMap.get(DcMotor.class, "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class, "dosGauche");

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);

        moteurAvantGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurAvantDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("✅ Robot", "Initialisé");
        telemetry.update();
    }

    /**
     * Avance pendant X secondes
     */
    private void avancer(double secondes) {
        telemetry.addData("🚀", "Avance pendant %.1f sec", secondes);
        telemetry.update();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < secondes) {
            moteurAvantGauche.setPower(VITESSE);
            moteurAvantDroit.setPower(VITESSE);
            moteurArriereGauche.setPower(VITESSE);
            moteurArriereDroit.setPower(VITESSE);

            telemetry.addData("⏱️ Temps écoulé", "%.1f / %.1f sec", runtime.seconds(), secondes);
            telemetry.update();
        }

        arretMoteurs();
    }

    /**
     * Recule pendant X secondes
     */
    private void reculer(double secondes) {
        telemetry.addData("🔙", "Recule pendant %.1f sec", secondes);
        telemetry.update();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < secondes) {
            moteurAvantGauche.setPower(-VITESSE);
            moteurAvantDroit.setPower(-VITESSE);
            moteurArriereGauche.setPower(-VITESSE);
            moteurArriereDroit.setPower(-VITESSE);

            telemetry.addData("⏱️ Temps écoulé", "%.1f / %.1f sec", runtime.seconds(), secondes);
            telemetry.update();
        }

        arretMoteurs();
    }

    /**
     * Arrête tous les moteurs
     */
    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
    }
}