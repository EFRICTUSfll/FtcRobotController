package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo testServo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Récupération du servo depuis la config Robot
        testServo = hardwareMap.get(Servo.class, "testServo");

        // Position neutre (juste après Init)
        testServo.setPosition(0.5);

        while (opModeInInit()) {
            telemetry.addData("Servo", "En position neutre (0.5)");
            telemetry.update();
        }

        waitForStart();

        telemetry.addLine("Début du test servo...");
        telemetry.update();

        // --- TEST 1 : Aller à gauche ---
        testServo.setPosition(0.0);
        telemetry.addLine("Position gauche : 0.0");
        telemetry.update();
        sleep(1000);

        // --- TEST 2 : Aller à droite ---
        testServo.setPosition(1.0);
        telemetry.addLine("Position droite : 1.0");
        telemetry.update();
        sleep(1000);

        // --- TEST 3 : Demi vitesse → animation douce gauche → droite ---
        for(double pos = 0.0; pos <= 1.0; pos += 0.01) {
            testServo.setPosition(pos);
            sleep(15); // 15ms = mouvement lent
        }

        telemetry.addLine("Test terminé !");
        telemetry.update();

        // Fin : on remet au centre
        testServo.setPosition(0.5);
        sleep(500);
    }
}
