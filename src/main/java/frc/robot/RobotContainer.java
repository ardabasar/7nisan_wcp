package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AlignToHubOdometry;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.auto.AutoAlignToTagCommand;
import frc.robot.commands.auto.TowerDriveCommand;
import frc.robot.commands.auto.TowerRotateCommand;
import frc.robot.commands.auto.VisionAutoSeedCommand;
import frc.robot.commands.auto.VisionCorrectCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * ROBOT CONTAINER - 2026 REBUILT
 * ============================================================================
 * Tum subsystem'lerin ve command binding'lerinin merkezi.
 *
 * SUBSYSTEM'LER:
 *   - CommandSwerveDrivetrain: 4 modul swerve (Caracal CANivore, CAN 1-8)
 *   - VisionSubsystem: Limelight MegaTag2 lokalizasyon
 *   - ShooterSubsystem: 3x TalonFX (CAN 9 +yon, CAN 10-11 -yon)
 *   - IntakeArmSubsystem: 1x TalonFX encoder pozisyon (CAN 12)
 *   - IntakeRollerSubsystem: 1x TalonFX roller (CAN 13)
 *   - HopperSubsystem: 1x TalonFX kayis (CAN 14)
 *   - FeederSubsystem: 1x TalonFX shooter beslemesi (CAN 15)
 *   - ClimbSubsystem: 1x TalonFX asma (CAN 16)
 *   - HoodSubsystem: 2x Servo pozisyon (PWM 3-4)
 *
 * CAN ID HARITASI:
 *   CANivore "Caracal" bus:
 *     Drive motorlari:  1, 3, 5, 7
 *     Steer motorlari:  2, 4, 6, 8
 *     CANcoders:        9, 10, 11, 12
 *     Pigeon2:          13
 *   rio bus:
 *     Shooter:          9 (+1 yon), 10 (-1 yon), 11 (-1 yon)
 *     Intake Arm:       12 (kol, encoder pozisyon)
 *     Intake Roller:    13 (silindir, 0.5 hiz)
 *     Hopper:           14 (kayis, -0.25 hiz)
 *     Feeder:           15 (shooter beslemesi, voltaj mantigi)
 *     Climb:            16 (+/-0.25 hiz)
 *   PWM:
 *     Hood servolar:    3 (sol), 4 (sag)
 * ============================================================================
 */
public class RobotContainer {

    // ========================================================================
    // SWERVE SABITLERI
    // ========================================================================
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    private static final double JOYSTICK_DEADBAND = 0.15;
    private static final double INPUT_CURVE_EXPONENT = 1.5;
    private static final double TRANSLATION_INPUT_RATE_LIMIT = 3.5; // joystick unit / s
    private static final double ROTATION_INPUT_RATE_LIMIT = 5.0;    // joystick unit / s

    // ========================================================================
    // SWERVE REQUEST'LER
    // ========================================================================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SlewRateLimiter xInputLimiter = new SlewRateLimiter(TRANSLATION_INPUT_RATE_LIMIT);
    private final SlewRateLimiter yInputLimiter = new SlewRateLimiter(TRANSLATION_INPUT_RATE_LIMIT);
    private final SlewRateLimiter rotInputLimiter = new SlewRateLimiter(ROTATION_INPUT_RATE_LIMIT);
    private boolean driveXYInverted = false;

    // ========================================================================
    // CONTROLLER
    // ========================================================================
    private final CommandXboxController joystick = new CommandXboxController(0);

    // ========================================================================
    // SUBSYSTEM'LER
    // ========================================================================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision         = new VisionSubsystem(drivetrain, "limelight");
    private final ShooterSubsystem shooter       = new ShooterSubsystem();
    private final HoodSubsystem hood             = new HoodSubsystem();
    private final FeederSubsystem feeder         = new FeederSubsystem();
    private final HopperSubsystem hopper         = new HopperSubsystem();
    private final IntakeArmSubsystem intakeArm   = new IntakeArmSubsystem();
    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    private final ClimbSubsystem climb           = new ClimbSubsystem();

    // ========================================================================
    // OTONOM
    // ========================================================================
    private final SendableChooser<Command> autoChooser;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public RobotContainer() {
        // Limelight kamera stream'ini dashboard'a gonder
        // Elastic'te "CameraServer" -> "limelight" olarak gorunur
        HttpCamera limelightCamera = new HttpCamera(
            "limelight",
            "http://10.95.45.11:5800/stream.mjpg",
            HttpCameraKind.kMJPGStreamer
        );
        CameraServer.startAutomaticCapture(limelightCamera);

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putBoolean("Drive/XYInverted", driveXYInverted);
        configureBindings();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    // ========================================================================
    // NAMED COMMANDS (PathPlanner Otonom icin)
    // ========================================================================
    private void registerNamedCommands() {
        // Atis: Shooter + Hood + Feeder + Hopper + IntakeArm agitasyon
        NamedCommands.registerCommand("shoot",
            new ShootCommand(shooter, hood, feeder, hopper, vision, "limelight", intakeArm)
                .withTimeout(5.0));

        // Intake: Arm + Roller
        NamedCommands.registerCommand("intake",
            new IntakeCommand(intakeArm, intakeRoller).withTimeout(3.0));

        // Hopper calistir
        NamedCommands.registerCommand("hopperRun",
            Commands.startEnd(() -> hopper.run(), () -> hopper.stop(), hopper).withTimeout(3.0));

        // Hizalama (614 tarzi odometry+hub yonu bazli)
        NamedCommands.registerCommand("alignToTag",
            new AlignToHubOdometry(drivetrain, vision).withTimeout(3.0));

        // Climb yukari (mekanizmayi uzat) - 3 saniye
        NamedCommands.registerCommand("climbUp",
            new ClimbCommand(climb, ClimbCommand.Direction.UP).withTimeout(2.0));

        // Climb asagi (robotu as) - 3 saniye
        NamedCommands.registerCommand("climbDown",
            new ClimbCommand(climb, ClimbCommand.Direction.DOWN).withTimeout(3.0));

        // ==============================================================
        // TOWER HIZALAMA - Asilma icin (Tag 15 Red / Tag 31 Blue)
        // 2 FAZLI: Once don+ortala, sonra duz ilerle
        // PathPlanner'da: path -> alignToTowerRot -> alignToTowerDrive -> climbUp
        // ==============================================================

        // FAZ 1: Sadece donus - tag'i ekranin ortasina getir (ileri/geri YOK)
        NamedCommands.registerCommand("alignToTowerRot",
            new TowerRotateCommand(drivetrain, "limelight", MaxAngularRate)
                .withTimeout(3.0));

        // FAZ 2: Sadece duz ilerle - 0.5m mesafeye yaklas (donus YOK)
        NamedCommands.registerCommand("alignToTowerDrive",
            new TowerDriveCommand(drivetrain, "limelight", MaxSpeed, 0.5)
                .withTimeout(3.0));

        // FAZ 2 alternatif: Yakin mesafe (0.35m)
        NamedCommands.registerCommand("alignToTowerDriveClose",
            new TowerDriveCommand(drivetrain, "limelight", MaxSpeed, 0.35)
                .withTimeout(3.0));

        // ==============================================================
        // VISION DUZELTME - Path aralarinda odometry duzeltme
        // Robot durur, tag gorur, odometry gunceller, devam eder
        // ==============================================================
        // Hizli duzeltme (0.15-0.5s) - cogu path arasi icin yeterli
        NamedCommands.registerCommand("visionCorrect",
            new VisionCorrectCommand(drivetrain, vision, 0.15, 0.5));

        // Uzun duzeltme (0.2-0.8s) - onemli path'lerden once (atis, tower)
        NamedCommands.registerCommand("visionCorrectLong",
            new VisionCorrectCommand(drivetrain, vision, 0.2, 0.8));

        // ==============================================================
        // INTAKE ROLLER - Path sirasinda top alma
        // PathPlanner'da deadline group ile path'e paralel calistir:
        //   path bitince (hedefe ulasinca) roller durur
        // ==============================================================
        NamedCommands.registerCommand("intakeRoller",
            Commands.startEnd(
                () -> {
                    intakeRoller.run();
                    hopper.run();
                },
                () -> {
                    intakeRoller.stop();
                    hopper.stop();
                },
                intakeRoller, hopper));

        // Sadece roller (hopper'siz)
        NamedCommands.registerCommand("intakeRollerOnly",
            Commands.startEnd(
                () -> intakeRoller.run(),
                () -> intakeRoller.stop(),
                intakeRoller));

        // Vision kontrol
        NamedCommands.registerCommand("visionOn", Commands.runOnce(() -> vision.setEnabled(true)));
        NamedCommands.registerCommand("visionOff", Commands.runOnce(() -> vision.setEnabled(false)));
    }

    /*
     * ========================================================================
     * XBOX CONTROLLER BUTON HARITASI - 2026 REBUILT
     * ========================================================================
     *
     *  STICKS (SADECE SURUS):
     *    Sol Stick       -> Swerve surme (field-centric X/Y)
     *    Sag Stick X     -> Donus (rotation)
     *
     *  FACE BUTONLARI:
     *    A (alt)         -> Hopper Manuel Ileri
     *    B (sag)         -> Intake Arm Manuel asagi indir (Yedek)
     *    X (sol)         -> Pas (shooter max hizla calissin)
     *    Y (ust)         -> Hopper Manuel Geri (Sikisma)
     *
     *  BUMPER / TRIGGER:
     *    RB              -> Hub yonune donus hizalama (basili tut, Auto-Aim)
     *    RT              -> ATIS! (Shooter+Feeder+Hood+Hopper)
     *    LB              -> Turbo (basili tut = max hiz, basilmiyorken %60)
     *    LT              -> FULL INTAKE (Kol asagi indir + Roller calistir)
     *
     *  D-PAD (POV):
     *    Up              -> Climb Yukari (Uzat)
     *    Down            -> Climb Asagi (Cek/Asil)
     *    Right           -> Intake yukari kaldirma
     *    Left            -> Feeder ve shooter geri hareket etsin
     *
     *  MENU:
     *    Back            -> Field-centric sifirla (heading reset)
     *    Start           -> Servo Test
     *
     * ========================================================================
     */
    private void configureBindings() {

        // ==================================================================
        // SWERVE SURME (varsayilan komut - LB turbo mantigi ile)
        // LB basili: max hiz, LB basilmiyorken: %60 hiz
        // ==================================================================
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                final double xySign = driveXYInverted ? -1.0 : 1.0;
                final double shapedForward = xySign * shapeInput(-joystick.getLeftY());
                final double shapedLeft = xySign * shapeInput(-joystick.getLeftX());
                final double shapedRotation = shapeInput(-joystick.getRightX());

                final double smoothedForward = xInputLimiter.calculate(shapedForward);
                final double smoothedLeft = yInputLimiter.calculate(shapedLeft);
                final double smoothedRotation = rotInputLimiter.calculate(shapedRotation);

                // LB basili mi? Turbo mod (max hiz) : Normal mod (%60 hiz)
                final double speedMultiplier = joystick.getHID().getLeftBumperButton() ? 1.0 : 0.3;

                return drive
                    .withVelocityX(smoothedForward * MaxSpeed * speedMultiplier)
                    .withVelocityY(smoothedLeft * MaxSpeed * speedMultiplier)
                    .withRotationalRate(smoothedRotation * MaxAngularRate);
            }));

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // ==================================================================
        // RT (Sag Trigger) -> ATIS
        //   Mesafe olc -> Shooter RPM + Hood aci ayarla
        //   Shooter + Feeder + Hopper birlikte calisir
        //   Intake arm'a DOKUNULMAZ, neredeyse orada kalir
        // ==================================================================
        joystick.rightTrigger(0.5).whileTrue(
            new ShootCommand(shooter, hood, feeder, hopper, vision, "limelight", intakeArm));

        // ==================================================================
        // RB -> Hub yonune donus hizalama (614 tarzi odometry bazli, basili tut)
        // Odometry uzerinden atan2 ile hub merkezine yon hesaplar.
        // AprilTag gorunmese bile calisir (vision seed yeterli).
        // ==================================================================
        joystick.rightBumper().whileTrue(
            new AlignToHubOdometry(drivetrain, vision,
                () -> {
                    double sign = driveXYInverted ? -1.0 : 1.0;
                    return sign * shapeInput(-joystick.getLeftY()) * MaxSpeed;
                },
                () -> {
                    double sign = driveXYInverted ? -1.0 : 1.0;
                    return sign * shapeInput(-joystick.getLeftX()) * MaxSpeed;
                }));

        // ==================================================================
        // A -> Hopper Manuel Ileri (basili tut = calistir)
        // ==================================================================
        joystick.a().whileTrue(
            Commands.startEnd(() -> hopper.run(), () -> hopper.stop(), hopper));

        // ==================================================================
        // B -> Intake Arm Manuel asagi indir (Yedek, basili tut = -0.25)
        // ==================================================================
        joystick.b().whileTrue(
            Commands.startEnd(
                () -> intakeArm.setSpeed(-IntakeArmSubsystem.ARM_SPEED),
                () -> intakeArm.stop(),
                intakeArm));

        // ==================================================================
        // LB -> TURBO: Surma hizi kontrolu swerve default command icinde
        //       (basilmiyorken %60, basili tutuluyorken %100)
        //       Burada ekstra binding yok, default command icinde halledildi
        // ==================================================================

        // ==================================================================
        // POV UP -> Climb Yukari (Uzat)
        // ==================================================================
        joystick.povUp().whileTrue(
            new ClimbCommand(climb, ClimbCommand.Direction.UP));

        // ==================================================================
        // POV DOWN -> Climb Asagi (Cek/Asil)
        // ==================================================================
        joystick.povDown().whileTrue(
            new ClimbCommand(climb, ClimbCommand.Direction.DOWN));

        // ==================================================================
        // POV RIGHT -> Intake yukari kaldirma (basili tut = +0.25)
        // ==================================================================
        joystick.povRight().whileTrue(
            Commands.startEnd(
                () -> intakeArm.setSpeed(IntakeArmSubsystem.ARM_SPEED),
                () -> intakeArm.stop(),
                intakeArm));

        // ==================================================================
        // POV LEFT -> Feeder, Shooter ve IntakeRoller geri hareket etsin
        // ==================================================================
        joystick.povLeft().whileTrue(
            Commands.startEnd(
                () -> {
                    shooter.setRPM(-1500);  // Shooter geri (negatif RPM)
                    feeder.reverse();        // Feeder geri
                    intakeRoller.reverse();  // IntakeRoller geri
                },
                () -> {
                    shooter.stop();
                    feeder.stop();
                    intakeRoller.stop();
                },
                shooter, feeder, intakeRoller));

        // ==================================================================
        // X -> Pas: Shooter max hizla calissin (basili tut)
        // ==================================================================
        joystick.x().whileTrue(
            Commands.startEnd(
                () -> shooter.setRPM(5040),  // Max shooter RPM
                () -> shooter.stop(),
                shooter));

        // ==================================================================
        // Y -> Hopper Manuel Geri (Sikisma durumunda)
        // ==================================================================
        joystick.y().whileTrue(
            Commands.startEnd(() -> hopper.reverse(), () -> hopper.stop(), hopper));

        // ==================================================================
        // LT -> INTAKE: Sadece Roller calistir (Arm sabit kalir)
        // ==================================================================
        joystick.leftTrigger(0.5).whileTrue(
            Commands.startEnd(
                () -> intakeRoller.run(),
                () -> intakeRoller.stop(),
                intakeRoller));

        // ==================================================================
        // Start -> SERVO TEST (basili tut = MAX, birak = DEFAULT)
        // ==================================================================
        joystick.start().whileTrue(
            Commands.startEnd(
                () -> hood.setPosition(HoodSubsystem.MAX_POSITION),  // 0.77
                () -> hood.setDefault(),                              // 0.5
                hood));

        // ==================================================================
        // Back -> HEADING RESET (Pigeon sifirla)
        //   Robotun su an baktigi yon = driver'a gore "ileri" olur.
        //   Field-centric suruste "ileri" her zaman driver'dan uzaga gider.
        // ==================================================================
        joystick.back().onTrue(Commands.runOnce(() -> {
            drivetrain.seedFieldCentric();
            xInputLimiter.reset(0);
            yInputLimiter.reset(0);
            SmartDashboard.putString("Drive/Status", "HEADING RESET!");
        }));

        // ==================================================================
        // TELEMETRI
        // ==================================================================
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * WCP benzeri joystick shaping:
     * 1) Deadband uygular
     * 2) Signed power ile merkezde daha ince kontrol verir
     */
    private static double shapeInput(double rawInput) {
        final double deadbanded = MathUtil.applyDeadband(rawInput, JOYSTICK_DEADBAND);
        return Math.copySign(Math.pow(Math.abs(deadbanded), INPUT_CURVE_EXPONENT), deadbanded);
    }

    /**
     * Autonomous command - vision seed destekli.
     */
    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) return Commands.print("Otonom secilmedi!");

        return Commands.sequence(
            new VisionAutoSeedCommand(drivetrain, vision, "limelight"),
            selected.asProxy()
        );
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================
    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public VisionSubsystem getVision() { return vision; }
    public ShooterSubsystem getShooter() { return shooter; }
    public HoodSubsystem getHood() { return hood; }
    public FeederSubsystem getFeeder() { return feeder; }
    public HopperSubsystem getHopper() { return hopper; }
    public IntakeArmSubsystem getIntakeArm() { return intakeArm; }
    public IntakeRollerSubsystem getIntakeRoller() { return intakeRoller; }
    public ClimbSubsystem getClimb() { return climb; }
}
