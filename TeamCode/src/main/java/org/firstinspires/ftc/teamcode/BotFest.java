package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * ════════════════════════════════════════════════════════════
 *  BRICKMAKERS — Team 31563
 *  BotFest — TeleOp National (Lyon)
 *
 *  ARCHITECTURE DEUX JOUEURS / DEUX MODES :
 *  ┌─────────────────────────────────────────────────────┐
 *  │  PARTIE BASSE  (gamepad1)  — Toujours manuel        │
 *  │    • Déplacement Mecanum (stick droit + gauche)     │
 *  │    • Vitesse dpad haut/bas                          │
 *  │    • Ramassage LB toggle                            │
 *  │    • Montage X toggle                               │
 *  ├─────────────────────────────────────────────────────┤
 *  │  PARTIE HAUTE  (gamepad1)  — FULL AUTO par défaut   │
 *  │    Turret    → suit AprilTag automatiquement        │
 *  │    Shooter   → vitesse calculée selon distance      │
 *  │    Angle     → calculé selon distance (max 110°)    │
 *  │    R1        → bascule en mode MANUEL haute partie  │
 *  │    En mode manuel haute :                           │
 *  │      L2/R2   → turret gauche/droite                 │
 *  │      Triangle → toggle shooter ON/OFF               │
 *  │      Cercle   → cycle preset (+1 à chaque appui)    │
 *  └─────────────────────────────────────────────────────┘
 *
 *  LED :
 *    🔵 Bleu    → initialisation
 *    ⚪ Off     → idle
 *    🟠 Orange  → ramassage actif
 *    🔴 Rouge   → shooter actif, vitesse pas encore stable
 *    🟢 Vert    → shooter PRÊT (appuie sur ○ pour monter les balles)
 *
 *  SHOOTER — GoBilda Yellow Jacket 6000 RPM (223-1 réduction ×1)
 *    Encodeur intégré : 28 ticks/tour (résolution moteur)
 *    Avec réduction 1:1 (direct drive sur flywheel) :
 *      1 RPM = 28 ticks/s  → vitesse_ticks = RPM × 28 / 60 × 28
 *    Si tu as une réduction différente, change TICKS_PAR_TOUR.
 *
 *  CALIBRATION PIDF SHOOTER — lis les commentaires dans configurerPIDShooter()
 *
 * ════════════════════════════════════════════════════════════
 */
@TeleOp(name = "MAX- TODAY", group = "National")
public class BotFest extends LinearOpMode {

    // ════════════════════════════════════════════════════════
    //  HARDWARE
    // ════════════════════════════════════════════════════════

    public Servo      light;

    DcMotor           moteurAvantGauche;
    DcMotor           moteurAvantDroit;
    DcMotor           moteurArriereGauche;
    DcMotor           moteurArriereDroit;
    DcMotorEx         shooter;
    DcMotor           intakeMoteur;
    DcMotor           montageGauche;
    DcMotor           montageDroit;

    CRServo           servoRamassageDroit;
    CRServo           servoRamassageGauche;
    CRServo           servoMoteurRamassageBalle;
    CRServo           servoTurret;
    Servo             servoAngleShooter;

    IMU               imu;
    Limelight3A       limelight;
    GoBildaPinpointDriver pinpoint;

    // ════════════════════════════════════════════════════════
    //  CONVERSION RPM ↔ TICKS/S
    //
    //  GoBilda Yellow Jacket 6000 RPM, encodeur 28 ticks/tour MOTEUR.
    //  Si ton flywheel est monté directement sur l'axe moteur (pas de
    //  réduction supplémentaire), alors :
    //      ticks/s = RPM × 28 / 60
    //
    //  Si tu as une réduction (ex: 2:1), multiplie TICKS_PAR_TOUR par
    //  le rapport de réduction.
    //
    //  ⚙️  POUR RECALIBRER : branche la Driver Station, regarde la
    //  télémétrie "Shooter RPM actuel". Si la valeur affichée ne
    //  correspond pas à la réalité mesurée, ajuste TICKS_PAR_TOUR.
    // ════════════════════════════════════════════════════════

    /** Ticks encoder par tour de flywheel. 28 = direct drive sur YJ. */
    private static final double TICKS_PAR_TOUR = 28.0;

    /** Convertit des RPM en ticks/s pour setVelocity(). */
    private double rpmEnTicks(double rpm) {
        return rpm * TICKS_PAR_TOUR / 60.0;
    }

    /** Convertit ticks/s en RPM pour l'affichage. */
    private double ticksEnRpm(double ticks) {
        return ticks * 60.0 / TICKS_PAR_TOUR;
    }

    // ════════════════════════════════════════════════════════
    //  PIDF SHOOTER
    //
    //  COMMENT RECALIBRER (lis ça avant de toucher aux valeurs) :
    //
    //  F (FeedForward) — règle ça EN PREMIER.
    //    F est la puissance de base pour maintenir une vitesse cible.
    //    Formule de départ : F = 32767 / vitesse_max_ticks
    //    vitesse_max_ticks pour 6000 RPM direct drive = 6000×28/60 = 2800 t/s
    //    F de départ = 32767 / 2800 ≈ 11.7
    //    Ajuste F jusqu'à ce que le shooter atteigne ~80% de la vitesse
    //    cible sans P. Si trop lent → monte F. Si ça dépasse → baisse F.
    //
    //  P (Proportionnel) — règle après F.
    //    Monte P progressivement. Le shooter doit atteindre la cible
    //    rapidement sans osciller. Si ça oscille → baisse P.
    //    Valeur de départ sûre : P = 10.
    //
    //  I (Intégral) — seulement si P seul ne rattrape pas l'erreur résiduelle.
    //    Monte I très doucement (par pas de 0.5). Trop de I → oscillations lentes.
    //    Si le shooter est stable à ±50 RPM avec P seul, laisse I = 0.
    //
    //  D (Dérivé) — amortit les oscillations causées par P.
    //    En général très petit. Monte par pas de 0.1.
    //    Trop de D sur un signal bruité → à-coups violents.
    //
    //  TOLERANCE_RPM : l'écart acceptable pour considérer "PRÊT".
    //    ±50 RPM est raisonnable pour un tir cohérent.
    //    Si tu veux plus de précision descends à ±30, mais le shooter
    //    sera moins souvent "PRÊT" (il faut attendre plus longtemps).
    // ════════════════════════════════════════════════════════

    private static final double SHOOTER_P         = 12.0;
    private static final double SHOOTER_I         =  0.0;  // à 0 pour commencer, monte si erreur résiduelle
    private static final double SHOOTER_D         =  0.3;
    private static final double SHOOTER_F         = 11.7;  // 32767 / 2800 ticks/s max

    /** Tolérance en RPM pour déclarer le shooter "PRÊT". */
    private static final double TOLERANCE_RPM     = 50.0;

    // ════════════════════════════════════════════════════════
    //  PRESETS DE TIR
    //
    //  Servo GoBilda 360° MODE ANGLE :
    //    0.0 = 0°  |  1.0 = 360°
    //    110° max → position servo max = 110/360 = 0.306
    //
    //  Vitesses en RPM (converties en ticks/s à l'usage).
    //
    //  Structure : { distanceCm, angleServo (0.0–0.306), RPM }
    //
    //  ⚠️  CES VALEURS SONT À AJUSTER SUR LE TERRAIN.
    //      La colonne distanceCm sert uniquement au mode AUTO
    //      (sélection automatique du preset selon ty Limelight).
    //      En mode manuel, cercle cycle parmi tous les presets.
    //
    //  COMMENT RECALIBRER LES PRESETS :
    //    1. Place le robot à la distance indiquée du panier.
    //    2. Lance en mode manuel (R1).
    //    3. Active le shooter (Triangle).
    //    4. Cycle les presets (Cercle) jusqu'au bon numéro.
    //    5. Tire. Si ça tombe court → monte RPM. Si ça dépasse → baisse RPM.
    //    6. Pour l'angle : si la balle part trop haut → monte angleServo
    //       (plus proche de 0.306 = plus à plat). Si trop bas → baisse.
    // ════════════════════════════════════════════════════════

    private static final double[][] PRESETS_TIR = {
            // { distanceCm, angleServo, RPM }
            //  angleServo MAX = 0.306 (= 110°, ne jamais dépasser)
            { 10.0,  0.30, 1200.0 },   // Preset  1 — ~10  cm
            { 30.0,  0.28, 1500.0 },   // Preset  2 — ~30  cm
            { 50.0,  0.26, 1800.0 },   // Preset  3 — ~50  cm
            { 70.0,  0.24, 2100.0 },   // Preset  4 — ~70  cm
            { 90.0,  0.22, 2400.0 },   // Preset  5 — ~90  cm
            {110.0,  0.20, 2700.0 },   // Preset  6 — ~110 cm
            {130.0,  0.18, 3000.0 },   // Preset  7 — ~130 cm
            {150.0,  0.15, 3300.0 },   // Preset  8 — ~150 cm
            {175.0,  0.11, 3600.0 },   // Preset  9 — ~175 cm
            {200.0,  0.07, 4000.0 },   // Preset 10 — ~200 cm
    };

    private int presetActif = 0; // index 0–9

    // ════════════════════════════════════════════════════════
    //  TURRET — SUIVI APRILTAG (PD)
    //
    //  ⚠️  CAMÉRA MONTÉE À L'ENVERS → txCorrige = -txBrut
    //
    //  TX_OFFSET_DEG : décalage de centre optique.
    //    Si le turret se stabilise mais pas centré sur le tag
    //    (il s'arrête à quelques degrés à côté), c'est que la
    //    caméra n'est pas dans l'axe du turret.
    //    → Mesure l'écart en degrés sur la télémétrie "tx corrigé"
    //      quand le turret est arrêté en face du tag.
    //    → Mets cette valeur dans TX_OFFSET_DEG (positif = décalage droite).
    //    Exemple : turret s'arrête quand tx = +3° → TX_OFFSET_DEG = 3.0
    //
    //  TURRET_DEADBAND_DEG : zone de tolérance autour du centre.
    //    Doit être ≥ bruit de mesure Limelight (~0.5°).
    //    Si le turret vibre légèrement même centré → monte deadband.
    //    Si tu veux une précision maximale → descends deadband (min ~1.0).
    // ════════════════════════════════════════════════════════

    private static final int    TURRET_TAG_ID       = -1;   // -1 = suit n'importe quel tag

    /**
     * Décalage en degrés à soustraire à tx après correction miroir.
     * Règle cette valeur si le turret se stabilise décalé du centre.
     * ⚙️  POUR RECALIBRER : place un tag en face, laisse le turret se
     * stabiliser, lis "Turret tx corrigé" sur la télémétrie quand il
     * est arrêté. Cette valeur = TX_OFFSET_DEG à entrer ici.
     */
    private static final double TX_OFFSET_DEG       =  0.0; // ← ajuste ici si décentré

    private static final double KP_TURRET           = 0.025;
    private static final double KD_TURRET           = 0.003;
    private static final double TURRET_DEADBAND_DEG = 2.5;
    private static final double TURRET_MIN_POWER    = 0.07;
    private static final double TURRET_MAX_POWER    = 0.60;

    private double      derniereTxTurret = 0.0;
    private ElapsedTime tempsTurret      = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  MODE HAUTE PARTIE
    //
    //  modeAutoHaut = true  → turret + shooter autonomes
    //  modeAutoHaut = false → contrôle manuel (L2/R2 + Triangle + Cercle)
    //  R1 bascule entre les deux modes.
    // ════════════════════════════════════════════════════════

    private boolean     modeAutoHaut    = true;  // FULL AUTO par défaut
    private ElapsedTime debounceR1      = new ElapsedTime();
    private ElapsedTime debounceMonter  = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT — RAMPE ACC/DEC
    // ════════════════════════════════════════════════════════

    private double vitesseDeplacement             = 0.7;
    private double puissanceAvantGaucheActuelle   = 0;
    private double puissanceAvantDroitActuelle    = 0;
    private double puissanceArriereGaucheActuelle = 0;
    private double puissanceArriereDroitActuelle  = 0;

    private static final double TAUX_ACCELERATION = 0.25;
    private static final double TAUX_DECELERATION = 0.35;
    private static final double SEUIL_MORT        = 0.05;

    // ════════════════════════════════════════════════════════
    //  ÉTATS BOOLÉENS
    // ════════════════════════════════════════════════════════

    private boolean shooterActif      = false;
    private boolean estRamassageActif = false;
    private boolean montageActif      = false;

    // ════════════════════════════════════════════════════════
    //  TIMERS DEBOUNCE
    // ════════════════════════════════════════════════════════

    private ElapsedTime tempsDebounceVitesse      = new ElapsedTime();
    private ElapsedTime tempsDebounceRamassage    = new ElapsedTime();
    private ElapsedTime tempsDebounceShooter      = new ElapsedTime();
    private ElapsedTime tempsDebonceMontage       = new ElapsedTime();
    private ElapsedTime tempsDebounceAngleShooter = new ElapsedTime();

    // ════════════════════════════════════════════════════════
    //  BOUCLE PRINCIPALE
    // ════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        initialisationDuRobot();
        configurerPIDShooter();

        light.setPosition(LED_BLEU);
        telemetry.addLine("✅ Brickmakers 31563 — Prêt !");
        telemetry.addLine("Mode : FULL AUTO haut | Manuel bas");
        telemetry.addLine("R1 = basculer mode haute partie");
        telemetry.update();

        waitForStart();
        tempsTurret.reset();

        while (opModeIsActive()) {

            // ── Bascule mode haute partie (R1) ────────────────
            if (debounceR1.milliseconds() > 300 && gamepad1.right_bumper) {
                modeAutoHaut = !modeAutoHaut;
                if (!modeAutoHaut) {
                    // En passant en manuel : arrêt shooter auto, turret stop
                    servoTurret.setPower(0);
                }
                debounceR1.reset();
            }

            // ── Partie basse — toujours manuelle ──────────────
            gestionVitesseDeplacement();
            deplacementFluide();
            gestionRamassage();
            gestionMontage();

            // ── Partie haute — auto ou manuel ─────────────────
            if (modeAutoHaut) {
                gestionHautAuto();
            } else {
                gestionHautManuel();
            }

            // ── LED ───────────────────────────────────────────
            mettreAJourLED();

            telemetry.addData("Mode haute partie", modeAutoHaut ? "🤖 AUTO (R1=manuel)" : "🕹️  MANUEL (R1=auto)");
            telemetry.update();
        }

        arretMoteurs();
    }

    // ════════════════════════════════════════════════════════
    //  PARTIE HAUTE — MODE AUTO
    //
    //  Le turret suit le tag en permanence.
    //  Le shooter et l'angle sont réglés automatiquement selon
    //  la distance estimée par Limelight (ty → distance).
    //  Le shooter est TOUJOURS actif en mode auto.
    //  Cercle → monter les balles (seulement si shooter PRÊT).
    // ════════════════════════════════════════════════════════

    private void gestionHautAuto() {
        LLResult result     = limelight.getLatestResult();
        boolean  tagValide  = (result != null && result.isValid() && result.getStaleness() <= 300);
        FiducialResult tag  = tagValide ? trouverTag(result) : null;

        // ── Turret : suivi automatique ─────────────────────────
        suivreTurret(tag);

        // ── Shooter : toujours actif en auto ──────────────────
        shooterActif = true;

        // ── Sélection auto du preset selon distance ────────────
        if (tag != null) {
            double distanceCm = estimerDistance(tag.getTargetYDegrees());
            presetActif = choisirPreset(distanceCm);
        }
        // Si pas de tag, on garde le dernier preset actif

        // Applique la vitesse en RPM
        shooter.setVelocity(rpmEnTicks(PRESETS_TIR[presetActif][2]));
        servoAngleShooter.setPosition(PRESETS_TIR[presetActif][1]);

        // ── Monter les balles : Cercle, seulement si shooter PRÊT ──
        boolean shooterPret = isShooterPret();
        if (shooterPret && debounceMonter.milliseconds() > 200 && gamepad1.circle) {
            monterBalle();
            debounceMonter.reset();
        }

        // ── Télémétrie ─────────────────────────────────────────
        double rpmActuel = ticksEnRpm(shooter.getVelocity());
        double rpmCible  = PRESETS_TIR[presetActif][2];
        telemetry.addData("Tag", tag != null ? String.format("ID %d", tag.getFiducialId()) : "Aucun");
        telemetry.addData("Preset auto", String.format("%d — %.0f cm", presetActif + 1, PRESETS_TIR[presetActif][0]));
        telemetry.addData("Shooter", String.format("%.0f / %.0f RPM  %s",
                rpmActuel, rpmCible, shooterPret ? "✅ PRÊT — appuie ○" : "⏳ ..."));
        telemetry.addData("Angle servo", String.format("%.3f (%.1f°)",
                PRESETS_TIR[presetActif][1], PRESETS_TIR[presetActif][1] * 360.0));
    }

    // ════════════════════════════════════════════════════════
    //  PARTIE HAUTE — MODE MANUEL
    //
    //  L2/R2   → turret gauche/droite
    //  Triangle → toggle shooter ON/OFF
    //  Cercle   → cycle preset +1
    // ════════════════════════════════════════════════════════

    private void gestionHautManuel() {
        // ── Turret manuel ──────────────────────────────────────
        float gachetteGauche = gamepad1.left_trigger;
        float gachetteDroit  = gamepad1.right_trigger;

        if (gachetteDroit > 0.1 || gachetteGauche > 0.1) {
            double puissance = 0;
            if (gachetteDroit  > 0.1) puissance =  gachetteDroit  * 0.5;
            if (gachetteGauche > 0.1) puissance = -gachetteGauche * 0.5;
            servoTurret.setPower(puissance);
            derniereTxTurret = 0;
        } else {
            servoTurret.setPower(0);
        }

        // ── Shooter toggle ─────────────────────────────────────
        if (tempsDebounceShooter.milliseconds() > 150 && gamepad1.triangle) {
            shooterActif = !shooterActif;
            tempsDebounceShooter.reset();
        }

        // ── Cycle preset ───────────────────────────────────────
        if (tempsDebounceAngleShooter.milliseconds() > 200 && gamepad1.circle) {
            presetActif = (presetActif + 1) % PRESETS_TIR.length;
            tempsDebounceAngleShooter.reset();
        }

        // ── Appliquer shooter ──────────────────────────────────
        if (shooterActif) {
            shooter.setVelocity(rpmEnTicks(PRESETS_TIR[presetActif][2]));
        } else {
            shooter.setVelocity(0);
        }
        servoAngleShooter.setPosition(PRESETS_TIR[presetActif][1]);

        // ── Monter les balles si shooter prêt ─────────────────
        // En mode manuel : même bouton cercle, mais on check d'abord si
        // on n'était pas en train de cycler. Ici on sépare :
        // un appui court = cycle preset. Long appui = monter balles.
        // Pour simplifier : si shooter actif ET prêt, right_bumper = monter.
        // (R1 est déjà utilisé pour le toggle mode, géré dans la boucle principale)

        // ── Télémétrie ─────────────────────────────────────────
        double rpmActuel = ticksEnRpm(shooter.getVelocity());
        double rpmCible  = PRESETS_TIR[presetActif][2];
        telemetry.addData("Preset", String.format("%d/10 — %.0f cm",
                presetActif + 1, PRESETS_TIR[presetActif][0]));
        telemetry.addData("Shooter", shooterActif
                ? String.format("%.0f / %.0f RPM  %s", rpmActuel, rpmCible,
                isShooterPret() ? "✅ PRÊT" : "⏳ ...")
                : "OFF");
        telemetry.addData("Angle servo", String.format("%.3f (%.1f°)",
                PRESETS_TIR[presetActif][1], PRESETS_TIR[presetActif][1] * 360.0));
    }

    // ════════════════════════════════════════════════════════
    //  SUIVI TURRET (commun auto et potentiellement réutilisable)
    // ════════════════════════════════════════════════════════

    private void suivreTurret(FiducialResult tag) {
        if (tag == null) {
            servoTurret.setPower(0);
            derniereTxTurret = 0;
            tempsTurret.reset();
            return;
        }

        // Correction caméra inversée + offset de centrage
        double txBrut    = tag.getTargetXDegrees();
        double txCorrige = -txBrut - TX_OFFSET_DEG;  // miroir + décalage calibration

        // PD
        double dt      = tempsTurret.seconds();
        double derivee = (dt > 0.001) ? (txCorrige - derniereTxTurret) / dt : 0;
        derniereTxTurret = txCorrige;
        tempsTurret.reset();

        double puissance = 0;
        if (Math.abs(txCorrige) > TURRET_DEADBAND_DEG) {
            puissance = KP_TURRET * txCorrige + KD_TURRET * derivee;

            if (puissance > 0 && puissance < TURRET_MIN_POWER)
                puissance = TURRET_MIN_POWER;
            else if (puissance < 0 && puissance > -TURRET_MIN_POWER)
                puissance = -TURRET_MIN_POWER;

            puissance = clamp(puissance, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        servoTurret.setPower(puissance);

        telemetry.addData("Turret tx brut",    String.format("%.1f°", txBrut));
        telemetry.addData("Turret tx corrigé", String.format("%.1f°  pwr=%.2f", txCorrige, puissance));
    }

    // ════════════════════════════════════════════════════════
    //  ESTIMATION DISTANCE via ty Limelight
    //
    //  ty = angle vertical du tag depuis le centre optique (degrés).
    //  Plus le tag est proche → ty plus grand (tag plus "en bas" dans
    //  l'image puisque la caméra est inclinée vers le bas).
    //
    //  Formule trigonométrique simplifiée :
    //    distance = hauteur_camera / tan(angle_camera + ty)
    //
    //  CAMERA_HAUTEUR_CM : hauteur de la Limelight depuis le sol (en cm).
    //  CAMERA_ANGLE_DEG  : angle d'inclinaison vers le bas de la caméra.
    //  TAG_HAUTEUR_CM    : hauteur du centre de l'AprilTag depuis le sol.
    //
    //  ⚙️  POUR RECALIBRER :
    //    Place le robot à une distance connue (ex: 100 cm) du tag.
    //    Lis "Distance estimée" sur la télémétrie.
    //    Ajuste CAMERA_HAUTEUR_CM et CAMERA_ANGLE_DEG jusqu'à concordance.
    // ════════════════════════════════════════════════════════

    /** Hauteur de la caméra Limelight au-dessus du sol (cm). */
    private static final double CAMERA_HAUTEUR_CM = 30.0;  // ← ajuste selon ton robot

    /** Inclinaison de la caméra vers le bas (degrés). */
    private static final double CAMERA_ANGLE_DEG  = 24.0;  // ← entre 21° et 28° selon toi

    /** Hauteur du centre de l'AprilTag depuis le sol (cm). */
    private static final double TAG_HAUTEUR_CM    = 20.0;  // ← ajuste selon le terrain

    private double estimerDistance(double tyDegres) {
        // Caméra inversée : ty est aussi miroir (vertical), donc -tyDegres
        double tyCorrige = -tyDegres;
        double angleTotal = Math.toRadians(CAMERA_ANGLE_DEG + tyCorrige);
        if (Math.abs(angleTotal) < 0.001) return 999.0; // éviter division par zéro
        double distance = (CAMERA_HAUTEUR_CM - TAG_HAUTEUR_CM) / Math.tan(angleTotal);
        telemetry.addData("Distance estimée", String.format("%.1f cm", distance));
        return Math.max(5.0, distance); // pas moins de 5 cm
    }

    /**
     * Choisit le preset dont la distance est la plus proche de distanceCm.
     */
    private int choisirPreset(double distanceCm) {
        int meilleur   = 0;
        double ecartMin = Double.MAX_VALUE;
        for (int i = 0; i < PRESETS_TIR.length; i++) {
            double ecart = Math.abs(PRESETS_TIR[i][0] - distanceCm);
            if (ecart < ecartMin) {
                ecartMin = ecart;
                meilleur = i;
            }
        }
        return meilleur;
    }

    // ════════════════════════════════════════════════════════
    //  SHOOTER — PIDF
    // ════════════════════════════════════════════════════════

    private void configurerPIDShooter() {
        // RUN_USING_ENCODER active le régulateur vitesse interne du SDK FTC.
        // Les coefficients PIDF sont appliqués en unités ticks/s.
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F)
        );
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Réduction du temps de réponse : le moteur répond aussi vite que possible.
        // Si tu vois des à-coups au démarrage, commente cette ligne.
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
    }

    /**
     * Vérifie si le shooter est stable à la vitesse cible (± TOLERANCE_RPM).
     */
    private boolean isShooterPret() {
        if (!shooterActif) return false;
        double rpmActuel = ticksEnRpm(shooter.getVelocity());
        double rpmCible  = PRESETS_TIR[presetActif][2];
        return Math.abs(rpmActuel - rpmCible) <= TOLERANCE_RPM;
    }

    /**
     * Déclenche la montée des balles (moteurs montage + servoMoteurRamassageBalle).
     * Appelé uniquement quand le shooter est PRÊT.
     */
    private void monterBalle() {
        servoMoteurRamassageBalle.setPower(-0.6);
        montageGauche.setPower(1.0);
        montageDroit.setPower(1.0);
    }

    // ════════════════════════════════════════════════════════
    //  LED — INDICATEUR D'ÉTAT
    //
    //  Priorité (du plus haut au plus bas) :
    //   1. Ramassage actif     → orange
    //   2. Shooter PRÊT        → vert  (appuie sur ○ pour tirer)
    //   3. Shooter actif/chauffe → rouge
    //   4. Mode auto           → bleu clignotant (géré simple ici : bleu fixe)
    //   5. Idle                → off
    // ════════════════════════════════════════════════════════

    private static final double LED_OFF    = 0.388;
    private static final double LED_VERT   = 0.500;
    private static final double LED_ROUGE  = 0.160;
    private static final double LED_ORANGE = 0.222;
    private static final double LED_BLEU   = 0.611;

    private void mettreAJourLED() {
        if (estRamassageActif) {
            light.setPosition(LED_ORANGE);
        } else if (shooterActif && isShooterPret()) {
            light.setPosition(LED_VERT);   // ← PRÊT : appuie sur ○ !
        } else if (shooterActif) {
            light.setPosition(LED_ROUGE);  // en train de monter en vitesse
        } else if (modeAutoHaut) {
            light.setPosition(LED_BLEU);   // auto actif, shooter off
        } else {
            light.setPosition(LED_OFF);
        }
    }

    // ════════════════════════════════════════════════════════
    //  DÉPLACEMENT FLUIDE
    // ════════════════════════════════════════════════════════

    public void deplacementFluide() {
        double forward = -gamepad1.right_stick_y;
        double right   =  gamepad1.right_stick_x;
        double rotate  =  gamepad1.left_stick_x;

        forward = appliquerZoneMorte(forward);
        right   = appliquerZoneMorte(right);
        rotate  = appliquerZoneMorte(rotate);

        double frontLeftPowerCible  = forward + right + rotate;
        double frontRightPowerCible = forward - right - rotate;
        double backRightPowerCible  = forward + right - rotate;
        double backLeftPowerCible   = forward - right + rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPowerCible),  Math.abs(frontRightPowerCible)),
                Math.max(Math.abs(backLeftPowerCible),   Math.abs(backRightPowerCible))
        );
        if (maxPower > 1.0) {
            frontLeftPowerCible  /= maxPower;
            frontRightPowerCible /= maxPower;
            backLeftPowerCible   /= maxPower;
            backRightPowerCible  /= maxPower;
        }

        puissanceAvantGaucheActuelle   = appliquerRampe(puissanceAvantGaucheActuelle,   frontLeftPowerCible);
        puissanceAvantDroitActuelle    = appliquerRampe(puissanceAvantDroitActuelle,    frontRightPowerCible);
        puissanceArriereGaucheActuelle = appliquerRampe(puissanceArriereGaucheActuelle, backLeftPowerCible);
        puissanceArriereDroitActuelle  = appliquerRampe(puissanceArriereDroitActuelle,  backRightPowerCible);

        moteurAvantGauche.setPower(vitesseDeplacement   * puissanceAvantGaucheActuelle);
        moteurAvantDroit.setPower(vitesseDeplacement    * puissanceAvantDroitActuelle);
        moteurArriereGauche.setPower(vitesseDeplacement * puissanceArriereGaucheActuelle);
        moteurArriereDroit.setPower(vitesseDeplacement  * puissanceArriereDroitActuelle);
    }

    // ════════════════════════════════════════════════════════
    //  GESTION VITESSE DE DÉPLACEMENT
    // ════════════════════════════════════════════════════════

    private void gestionVitesseDeplacement() {
        if (tempsDebounceVitesse.milliseconds() > 300) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                vitesseDeplacement = Math.min(vitesseDeplacement + 0.1, 1.0);
                tempsDebounceVitesse.reset();
            }
            if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                vitesseDeplacement = Math.max(vitesseDeplacement - 0.1, 0.1);
                tempsDebounceVitesse.reset();
            }
        }
        telemetry.addData("Vitesse déplacement", String.format("%.0f%%", vitesseDeplacement * 100));
    }

    // ════════════════════════════════════════════════════════
    //  RAMASSAGE
    // ════════════════════════════════════════════════════════

    private void gestionRamassage() {
        if (tempsDebounceRamassage.milliseconds() > 200 && gamepad1.left_bumper) {
            estRamassageActif = !estRamassageActif;
            tempsDebounceRamassage.reset();
        }

        if (estRamassageActif) {
            intakeMoteur.setPower(1);
            servoMoteurRamassageBalle.setPower(-1.0);
            servoRamassageGauche.setPower(-0.3);
            servoRamassageDroit.setPower(0.3);
            montageGauche.setPower(-1.0);
            montageDroit.setPower(-1.0);
        } else {
            intakeMoteur.setPower(0.0);
            servoMoteurRamassageBalle.setPower(0.0);
        }
    }

    // ════════════════════════════════════════════════════════
    //  MONTAGE
    // ════════════════════════════════════════════════════════

    private void gestionMontage() {
        if (tempsDebonceMontage.milliseconds() > 400 && gamepad1.cross) {
            montageActif = !montageActif;
            tempsDebonceMontage.reset();
        }

        if (montageActif) {
            servoRamassageGauche.setPower(0.3);
            servoRamassageDroit.setPower(-0.3);
            servoMoteurRamassageBalle.setPower(-0.6);
            montageGauche.setPower(1.0);
            montageDroit.setPower(1.0);
            intakeMoteur.setPower(1.0);
        } else {
            servoRamassageGauche.setPower(0.0);
            servoRamassageDroit.setPower(0.0);
            servoMoteurRamassageBalle.setPower(0.0);
            montageGauche.setPower(0.0);
            montageDroit.setPower(0.0);
        }
    }

    // ════════════════════════════════════════════════════════
    //  ARRÊT PROPRE
    // ════════════════════════════════════════════════════════

    private void arretMoteurs() {
        moteurAvantGauche.setPower(0);
        moteurAvantDroit.setPower(0);
        moteurArriereGauche.setPower(0);
        moteurArriereDroit.setPower(0);
        shooter.setVelocity(0);
        intakeMoteur.setPower(0);
        montageDroit.setPower(0);
        montageGauche.setPower(0);
        servoMoteurRamassageBalle.setPower(0);
        servoTurret.setPower(0);
        servoRamassageDroit.setPower(0);
        servoRamassageGauche.setPower(0);
        servoAngleShooter.setPosition(PRESETS_TIR[0][1]);
        light.setPosition(LED_OFF);
        limelight.stop();
    }

    // ════════════════════════════════════════════════════════
    //  INITIALISATION
    // ════════════════════════════════════════════════════════

    private void initialisationDuRobot() {
        moteurAvantDroit    = hardwareMap.get(DcMotor.class,   "avantDroit");
        moteurAvantGauche   = hardwareMap.get(DcMotor.class,   "avantGauche");
        moteurArriereDroit  = hardwareMap.get(DcMotor.class,   "dosDroit");
        moteurArriereGauche = hardwareMap.get(DcMotor.class,   "dosGauche");
        shooter             = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMoteur        = hardwareMap.get(DcMotor.class,   "intake");
        montageGauche       = hardwareMap.get(DcMotor.class,   "montageG");
        montageDroit        = hardwareMap.get(DcMotor.class,   "montageD");

        servoMoteurRamassageBalle = hardwareMap.get(CRServo.class, "ramassage");
        servoRamassageDroit       = hardwareMap.get(CRServo.class, "ramassageD");
        servoRamassageGauche      = hardwareMap.get(CRServo.class, "ramassageG");
        servoTurret               = hardwareMap.get(CRServo.class, "turret");
        servoAngleShooter         = hardwareMap.get(Servo.class,   "angleShooter");
        light                     = hardwareMap.get(Servo.class,   "light");

        imu      = hardwareMap.get(IMU.class,                    "imu");
        limelight = hardwareMap.get(Limelight3A.class,           "limelight");
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, "odoComputer");

        moteurAvantDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurAvantGauche.setDirection(DcMotor.Direction.FORWARD);
        moteurArriereDroit.setDirection(DcMotor.Direction.REVERSE);
        moteurArriereGauche.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intakeMoteur.setDirection(DcMotor.Direction.REVERSE);
        montageGauche.setDirection(DcMotor.Direction.FORWARD);
        montageDroit.setDirection(DcMotorSimple.Direction.REVERSE);

        moteurAvantGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurAvantDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereGauche.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        moteurArriereDroit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moteurAvantGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurAvantDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereGauche.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moteurArriereDroit.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        /*
        pinpoint.setOffsets(0, 0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        */

        servoAngleShooter.setPosition(PRESETS_TIR[0][1]);
        light.setPosition(LED_OFF);
    }

    // ════════════════════════════════════════════════════════
    //  UTILITAIRES
    // ════════════════════════════════════════════════════════

    private FiducialResult trouverTag(LLResult result) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        FiducialResult meilleur   = null;
        double         meilleurTx = Double.MAX_VALUE;

        for (FiducialResult f : fiducials) {
            boolean match = (TURRET_TAG_ID == -1) || (f.getFiducialId() == TURRET_TAG_ID);
            if (match && Math.abs(f.getTargetXDegrees()) < meilleurTx) {
                meilleurTx = Math.abs(f.getTargetXDegrees());
                meilleur   = f;
            }
        }
        return meilleur;
    }

    private double appliquerRampe(double actuelle, double cible) {
        if (cible == 0 && Math.abs(actuelle) < 0.05) return 0;
        double taux = (cible == 0) ? TAUX_DECELERATION : TAUX_ACCELERATION;
        return actuelle + (cible - actuelle) * taux;
    }

    private double appliquerZoneMorte(double valeur) {
        return (Math.abs(valeur) < SEUIL_MORT) ? 0 : valeur;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
