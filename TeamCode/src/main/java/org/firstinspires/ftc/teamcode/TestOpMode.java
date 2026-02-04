package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class TestOpMode extends LinearOpMode {
    // Déclaration des moteurs
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;

    // Variable pour le contrôle de vitesse
    private double vitesseMax = 0.7; // Vitesse par défaut à 70%

    @Override
    public void runOpMode() {
        // Initialisation des moteurs depuis la configuration hardware
        frontRightDrive = hardwareMap.get(DcMotor.class, "avantDroit");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "avantGauche");
        backRightDrive = hardwareMap.get(DcMotor.class, "dosDroit");
        backLeftDrive = hardwareMap.get(DcMotor.class, "dosGauche");

        // Configuration des directions des moteurs
        // Ajuster ces directions selon le comportement réel de votre robot
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Message d'initialisation
        telemetry.addData("Status", "Initialisé - Prêt à démarrer");
        telemetry.addData("Vitesse actuelle", "%.0f%%", vitesseMax * 100);
        telemetry.addData("Configuration", "Joystick G: Rotation | Joystick D: Direction");
        telemetry.update();

        // Attente du début du match (bouton PLAY)
        waitForStart();

        // Boucle principale pendant le match
        while (opModeIsActive()) {
            // ===========================================
            // SECTION 1: GESTION DE LA VITESSE AVEC D-PAD
            // ===========================================

            // Flèche du HAUT : Augmenter la vitesse de 10%
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                vitesseMax += 0.1;
                if (vitesseMax > 1.0) vitesseMax = 1.0; // Limite à 100%
                sleep(200); // Délai pour éviter les changements trop rapides
            }

            // Flèche du BAS : Réduire la vitesse de 10%
            if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                vitesseMax -= 0.1;
                if (vitesseMax < 0.1) vitesseMax = 0.1; // Minimum 10%
                sleep(200); // Délai pour éviter les changements trop rapides
            }

            // ===========================================
            // SECTION 2: LECTURE DES JOYSTICKS (INVERSÉS)
            // ===========================================

            // Joystick GAUCHE : Contrôle de ROTATION
            double rotation = gamepad1.left_stick_x;           // Rotation du robot

            // Joystick DROIT : Contrôle de DIRECTION
            double avancerReculer = -gamepad1.right_stick_y;   // Y inversé (haut = positif)
            double gaucheDroite = gamepad1.right_stick_x;      // Déplacement latéral

            // ===========================================
            // SECTION 3: CALCUL MECANUM (Cinématique)
            // ===========================================

            // Formules pour le mouvement Mecanum
            double puissanceAvantGauche = avancerReculer + gaucheDroite + rotation;
            double puissanceAvantDroit = avancerReculer - gaucheDroite - rotation;
            double puissanceArriereGauche = avancerReculer - gaucheDroite + rotation;
            double puissanceArriereDroit = avancerReculer + gaucheDroite - rotation;

            // ===========================================
            // SECTION 4: NORMALISATION DES PUISSANCES
            // ===========================================

            // Trouver la puissance maximale absolue
            double maxPuissance = Math.max(
                    Math.max(Math.abs(puissanceAvantGauche), Math.abs(puissanceAvantDroit)),
                    Math.max(Math.abs(puissanceArriereGauche), Math.abs(puissanceArriereDroit))
            );

            // Si une puissance dépasse 1.0, normaliser toutes les puissances
            if (maxPuissance > 1.0) {
                puissanceAvantGauche /= maxPuissance;
                puissanceAvantDroit /= maxPuissance;
                puissanceArriereGauche /= maxPuissance;
                puissanceArriereDroit /= maxPuissance;
            }

            // Application du multiplicateur de vitesse globale
            puissanceAvantGauche *= vitesseMax;
            puissanceAvantDroit *= vitesseMax;
            puissanceArriereGauche *= vitesseMax;
            puissanceArriereDroit *= vitesseMax;

            // ===========================================
            // SECTION 5: APPLICATION AUX MOTEURS
            // ===========================================

            frontLeftDrive.setPower(puissanceAvantGauche);
            frontRightDrive.setPower(puissanceAvantDroit);
            backLeftDrive.setPower(puissanceArriereGauche);
            backRightDrive.setPower(puissanceArriereDroit);

            // ===========================================
            // SECTION 6: AFFICHAGE TÉLÉMÉTRIE
            // ===========================================

            telemetry.addLine("=== CONTRÔLES MECANUM ===");
            telemetry.addData("Configuration", "Joystick G: Rotation | Joystick D: Direction");
            telemetry.addData("Joystick DROIT (Direction)", "Avancer: %.2f, Latéral: %.2f",
                    avancerReculer, gaucheDroite);
            telemetry.addData("Joystick GAUCHE (Rotation)", "Rotation: %.2f", rotation);
            telemetry.addLine("--- Puissances Moteurs ---");
            telemetry.addData("Moteurs", "AV-G: %.2f, AV-D: %.2f, AR-G: %.2f, AR-D: %.2f",
                    puissanceAvantGauche, puissanceAvantDroit,
                    puissanceArriereGauche, puissanceArriereDroit);
            telemetry.addLine("--- CONTRÔLE VITESSE ---");
            telemetry.addData("Vitesse", "%.0f%% (D-pad ↑/↓ pour ajuster)", vitesseMax * 100);
            telemetry.addData("Instructions", "Joystick D: Direction | Joystick G: Rotation");
            telemetry.update();
        }

        // ===========================================
        // SECTION 7: ARRÊT PROPRE EN FIN DE MATCH
        // ===========================================

        // Arrêt de tous les moteurs quand le programme se termine
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        telemetry.addData("Status", "Programme terminé");
        telemetry.update();
    }
}