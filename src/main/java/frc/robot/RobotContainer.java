package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

import static edu.wpi.first.units.Units.Meters;

import frc.robot.commands.auto.AutoAlignToTagCommand;
import frc.robot.commands.auto.TowerDriveCommand;
import frc.robot.commands.auto.TowerRotateCommand;
import frc.robot.commands.auto.VisionAutoSeedCommand;
import frc.robot.commands.auto.AutoShootCommand;
import frc.robot.commands.auto.VisionCorrectCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;


import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
* ============================================================================
* Bismillahirrahmanirrahim
*
* Ayet-el Kürsi (Bakara Suresi 255. Ayet)
*
* Allahu la ilahe illa huvel hayyul kayyum.
* La te'huzuhu sinetun ve la nevm.
* Lehu ma fis semavati ve ma fil ard.
* Men zellezi yeşfeu indehu illa bi iznih.
* Ya'lemu ma beyne eydihim ve ma halfehum.
* Ve la yuhitune bi şey'in min ilmihi illa bi ma şa'.
* Vesia kursiyyuhus semavati vel ard.
* Ve la yeuduhu hıfzuhuma.
* Ve huvel aliyyul azim.
* ============================================================================
*/

/**
 * ============================================================================
 * ROBOT CONTAINER - 2026 REBUILT
 * ============================================================================
 * Tum subsystem'lerin ve command binding'lerinin merkezi.
 *
 * SUBSYSTEM'LER:
 * - CommandSwerveDrivetrain: 4 modul swerve (Caracal CANivore, CAN 1-8)
 * - VisionSubsystem: Limelight MegaTag2 lokalizasyon
 * - ShooterSubsystem: 3x TalonFX (CAN 9 +yon, CAN 10-11 -yon)
 * - IntakeArmSubsystem: 1x TalonFX encoder pozisyon (CAN 12)
 * - IntakeRollerSubsystem: 1x TalonFX roller (CAN 13)
 * - HopperSubsystem: 1x TalonFX kayis (CAN 14)
 * - FeederSubsystem: 1x TalonFX shooter beslemesi (CAN 15)

 * - HoodSubsystem: 2x Servo pozisyon (PWM 3-4)
 *
 * CAN ID HARITASI:
 * CANivore "Caracal" bus:
 * Drive motorlari: 1, 3, 5, 7
 * Steer motorlari: 2, 4, 6, 8
 * CANcoders: 9, 10, 11, 12
 * Pigeon2: 13
 * rio bus:
 * Shooter: 9 (+1 yon), 10 (-1 yon), 11 (-1 yon)
 * Intake Arm: 12 (kol, encoder pozisyon)
 * Intake Roller: 13 (silindir, 0.5 hiz)
 * Hopper: 14 (kayis, -0.25 hiz)
 * Feeder: 15 (shooter beslemesi, voltaj mantigi)

 * PWM:
 * Hood servolar: 3 (sol), 4 (sag)
 * ============================================================================
 */
public class RobotContainer {

    // ========================================================================
    // SWERVE SABITLERI
    // ========================================================================
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    private static final double JOYSTICK_DEADBAND = 0.15;
    private static final double INPUT_CURVE_EXPONENT = 1.0; // 1.0 = lineer (curve kaldirildi)
    // ========================================================================
    // SWERVE REQUEST'LER
    // ========================================================================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private boolean driveXYInverted = false;

    // ========================================================================
    // CONTROLLER
    // ========================================================================
    private final CommandXboxController joystick = new CommandXboxController(0);

    // ========================================================================
    // SUBSYSTEM'LER
    // ========================================================================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain, "limelight");
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();
    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();

    private final LEDSubsystem leds = new LEDSubsystem();

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
                HttpCameraKind.kMJPGStreamer);
        CameraServer.startAutomaticCapture(limelightCamera);

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putBoolean("Drive/XYInverted", driveXYInverted);
        configureBindings();

        // ==================================================================
        // HOOD DEFAULT COMMAND - Kendi alaninda mesafeye gore ayarla
        // 2026 REBUILT saha: 16.518m x 8.043m
        //   Blue Alliance Bolgesi: x < 6.5m  (hub x=4.625)
        //   Neutral Zone:         x = 6.5 - 10.0m
        //   Red Alliance Bolgesi: x > 10.0m  (hub x=11.915)
        //
        // Kendi alaninda = hood mesafeye gore surekli ayarlanir
        // Neutral zone / karsi alan = hood sabit default, servo kipirrrtdamaz
        // RT basinca ShootCommand hood'u devralir (require override).
        // ==================================================================
        hood.setDefaultCommand(hood.run(() -> {
            double robotX = drivetrain.getState().Pose.getX();
            boolean inOwnZone;

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    inOwnZone = robotX < 6.5; // Blue alan
                } else {
                    inOwnZone = robotX > 10.0; // Red alan
                }
            } else {
                // Alliance bilinmiyorsa mesafe bazli fallback
                inOwnZone = vision.getDistanceToOwnHub() < 7.0;
            }

            if (inOwnZone) {
                double dist = vision.getDistanceToOwnHub();
                if (dist >= 0 && dist < 7.0) {
                    double hoodPos = ShootCommand.getHoodPositionForDistance(Meters.of(dist));
                    hood.setPosition(hoodPos);
                }
            } else {
                hood.setDefault();
            }
        }));
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    // ========================================================================
    // NAMED COMMANDS (PathPlanner Otonom icin)
    // ========================================================================
    private void registerNamedCommands() {
        // Atis (teleop): Shooter + Hood + Feeder + Hopper + IntakeArm agitasyon
        NamedCommands.registerCommand("shoot",
                new ShootCommand(shooter, hood, feeder, hopper, vision, "limelight", intakeArm)
                        .withTimeout(5.0));

        // Atis (otonom): Ayni mantik ama otomatik bitis (~3s)
        // Spin-up + 2.5s besleme, max 4s guvenlik timeout
        NamedCommands.registerCommand("autoShoot",
                new AutoShootCommand(shooter, hood, feeder, hopper, vision, "limelight", intakeArm));

        // Intake: Arm + Roller
        NamedCommands.registerCommand("intake",
                new IntakeCommand(intakeArm, intakeRoller).withTimeout(3.0));

        // Hopper calistir
        NamedCommands.registerCommand("hopperRun",
                Commands.startEnd(() -> hopper.run(), () -> hopper.stop(), hopper).withTimeout(3.0));

        // Hizalama (614 tarzi odometry+hub yonu bazli)
        NamedCommands.registerCommand("alignToTag",
                new AlignToHubOdometry(drivetrain, vision).withTimeout(0.5));


        // ==============================================================
        // TOWER HIZALAMA - Asilma icin (Tag 15 Red / Tag 31 Blue)
        // 2 FAZLI: Once don+ortala, sonra duz ilerle
        // PathPlanner'da: path -> alignToTowerRot -> alignToTowerDrive
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
        // path bitince (hedefe ulasinca) roller durur
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

        // Intake BASLAT - sadece roller, hopper YOK
        NamedCommands.registerCommand("intakeStart",
                Commands.runOnce(() -> {
                    intakeRoller.run();
                }));

        // Intake DURDUR - sadece roller
        NamedCommands.registerCommand("intakeStop",
                Commands.runOnce(() -> {
                    intakeRoller.stop();
                }));

        // Vision kontrol
        NamedCommands.registerCommand("visionOn", Commands.runOnce(() -> vision.setEnabled(true)));
        NamedCommands.registerCommand("visionOff", Commands.runOnce(() -> vision.setEnabled(false)));

        // Tumsek cikisi konum duzeltme - PathPlanner Event Marker ile tetiklenir
        NamedCommands.registerCommand("KonumDuzelt", Commands.runOnce(() -> vision.forceVisionUpdate()));

        // Shooter onceden dondur - nzr_7 Event Marker ile tetikle, spin-up suresi
        // kazandirir
        NamedCommands.registerCommand("spinUp", Commands.runOnce(() -> {
            shooter.setRPM(3500);
        }));

        // Intake arm indir - otonom basinda kolu ac
        NamedCommands.registerCommand("intakeDown",
                Commands.startEnd(
                        () -> intakeArm.setSpeed(-IntakeArmSubsystem.ARM_SPEED),
                        () -> intakeArm.stop(),
                        intakeArm).withTimeout(0.50));

        // Intake arm durdur
        NamedCommands.registerCommand("intakeArmStop",
                Commands.runOnce(() -> intakeArm.stop()));

        // Hub mesafe kontrolu — robot hub'a yeterince yakin degilse atislamasin
        // PathPlanner'da: alignToTag → shootIfClose seklinde kullan
        // Max 3m mesafeden atis yapar, daha uzaksa skip
        NamedCommands.registerCommand("shootIfClose",
                Commands.either(
                        // Hub'a yakin → atis yap
                        new AutoShootCommand(shooter, hood, feeder, hopper, vision, "limelight", intakeArm),
                        // Hub'a uzak → sadece uyari, zaman kaybetme
                        Commands.runOnce(() -> {
                            SmartDashboard.putString("Auto/Status", "SKIP:TooFar");
                        }),
                        // Kosul: hub mesafesi 3m'den az mi?
                        () -> vision.getDistanceToOwnHub() < 3.0));
    }

    /*
     * ========================================================================
     * XBOX CONTROLLER BUTON HARITASI - 2026 REBUILT
     * ========================================================================
     *
     * STICKS (SADECE SURUS):
     * Sol Stick -> Swerve surme (field-centric X/Y)
     * Sag Stick X -> Donus (rotation)
     *
     * FACE BUTONLARI:
     * A (alt) -> Hopper Manuel Ileri
     * B (sag) -> Intake Arm Manuel asagi indir (Yedek)
     * X (sol) -> Shooter spin-up (sadece shooter dondur)
     * Y (ust) -> Hopper Manuel Geri (Sikisma)
     *
     * BUMPER / TRIGGER:
     * RB -> Hub yonune donus hizalama (basili tut, SADECE nisan - atissiz)
     * RT -> SHOOT ON THE MOVE (Aim+Shoot+Drive paralel, 6237 sistemi)
     * LB -> PAS (Intake+Hopper+Feeder+Shooter hepsi birden, basili tut)
     * LT -> FULL INTAKE (Roller calistir)
     *
     * D-PAD (POV):
     * Up -> (bos)
     * Down -> (bos)
     * Right -> Intake yukari kaldirma
     * Left -> Feeder ve shooter geri hareket etsin
     *
     * MENU:
     * Back -> Field-centric sifirla (heading reset)
     * Start -> Servo Test
     *
     * ========================================================================
     */
    private void configureBindings() {

        // ==================================================================
        // SWERVE SURME (sabit %50 max hiz)
        // ==================================================================
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    final double rawForward = -joystick.getLeftY();
                    final double rawLeft = -joystick.getLeftX();
                    final double rawRotation = -joystick.getRightX();

                    final double xySign = driveXYInverted ? -1.0 : 1.0;
                    final double shapedForward = xySign * shapeInput(rawForward);
                    final double shapedLeft = xySign * shapeInput(rawLeft);
                    final double shapedRotation = shapeInput(rawRotation);

                    return drive
                            .withVelocityX(shapedForward * MaxSpeed)
                            .withVelocityY(shapedLeft * MaxSpeed)
                            .withRotationalRate(shapedRotation * MaxAngularRate);
                }));

        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // ==================================================================
        // RT (Sag Trigger) -> SHOOT ON THE MOVE (6237 sistemi)
        // Paralel calisir:
        //   1) AlignToHubOdometry: Hub'a kilitlen + surucu X/Y ile surmeye devam
        //   2) ShootCommand: Mesafe olc, RPM+Hood ayarla, aimed olunca feed
        // RT basili tut = sur + nishan al + hazir olunca otomatik at
        // ==================================================================
        joystick.rightTrigger(0.5).whileTrue(
                Commands.parallel(
                        new AlignToHubOdometry(drivetrain, vision,
                                () -> {
                                    double sign = driveXYInverted ? -1.0 : 1.0;
                                    return sign * shapeInput(-joystick.getLeftY()) * MaxSpeed;
                                },
                                () -> {
                                    double sign = driveXYInverted ? -1.0 : 1.0;
                                    return sign * shapeInput(-joystick.getLeftX()) * MaxSpeed;
                                }),
                        new ShootCommand(shooter, hood, feeder, hopper, vision, "limelight", intakeArm,
                                () -> AlignToHubOdometry.isAimed(drivetrain))));

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
        // LB -> PAS: Ortadan kendi tarafa top firlatma (~4-5m)
        // Sabit RPM 5000 + Hood 0.64 + Intake + Feeder + Hopper
        // Basili tut: alir almaz firlatir, surus etkilenmez
        // ==================================================================
        joystick.leftBumper().whileTrue(
                Commands.parallel(
                        Commands.startEnd(
                                () -> {
                                    shooter.setPercentOutput(1.0); // Tam 12V
                                    hood.setPosition(0.50);
                                    hopper.setSpeed(-1.0);          // Tam 12V (-1 yon)
                                    feeder.setPercentOutput(1.0);   // Tam 12V
                                },
                                () -> {
                                    shooter.stop();
                                    hood.setDefault();
                                    hopper.stop();
                                    feeder.stop();
                                },
                                shooter, hood, hopper, feeder),
                        // Intake arm agitasyonu (ShootCommand ile ayni mantik)
                        Commands.runEnd(
                                () -> {
                                    double phase = (Timer.getFPGATimestamp() % 0.80);
                                    intakeArm.setSpeed(phase < 0.40 ? 0.25 : -0.25);
                                },
                                () -> intakeArm.stop(),
                                intakeArm)));

        // ==================================================================
        // POV UP -> (bos)
        // ==================================================================

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
                            shooter.setRPM(-1500); // Shooter geri (negatif RPM)
                            feeder.reverse(); // Feeder geri
                            intakeRoller.reverse(); // IntakeRoller geri
                        },
                        () -> {
                            shooter.stop();
                            feeder.stop();
                            intakeRoller.stop();
                        },
                        shooter, feeder, intakeRoller));

        // ==================================================================
        // X -> Shooter spin-up (sadece shooter dondur, basili tut)
        // ==================================================================
        joystick.x().whileTrue(
                Commands.startEnd(
                        () -> shooter.setRPM(5040), // Max shooter RPM
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
                        () -> hood.setPosition(HoodSubsystem.MAX_POSITION), // 0.77
                        () -> hood.setDefault(), // 0.5
                        hood));

        // ==================================================================
        // Back -> HEADING RESET (Pigeon sifirla)
        // Robotun su an baktigi yon = driver'a gore "ileri" olur.
        // Field-centric suruste "ileri" her zaman driver'dan uzaga gider.
        // ==================================================================
        joystick.back().onTrue(Commands.runOnce(() -> {
            // Robotun SU AN baktigi yon = "ileri" olarak ayarla
            // Bu CTRE'nin operator perspective'ini kullanir - en dogru yontem
            drivetrain.seedFieldCentric();

            // Pigeon heading'i de sifirla (robotun su anki yonu = 0°)
            Pose2d currentPose = drivetrain.getState().Pose;
            drivetrain.resetPose(new Pose2d(
                    currentPose.getTranslation(),
                    new Rotation2d(0)));

            // Vision'i da resetle - tag gorurse yeniden dogru konum alsin
            vision.resetVisionSeed();
            SmartDashboard.putString("Drive/Status", "GYRO + VISION RESET!");
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
        if (selected == null)
            return Commands.print("Otonom secilmedi!");

        return Commands.sequence(
                new VisionAutoSeedCommand(drivetrain, vision, "limelight"),
                selected.asProxy());
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public VisionSubsystem getVision() {
        return vision;
    }

    public ShooterSubsystem getShooter() {
        return shooter;
    }

    public HoodSubsystem getHood() {
        return hood;
    }

    public FeederSubsystem getFeeder() {
        return feeder;
    }

    public HopperSubsystem getHopper() {
        return hopper;
    }

    public IntakeArmSubsystem getIntakeArm() {
        return intakeArm;
    }

    public IntakeRollerSubsystem getIntakeRoller() {
        return intakeRoller;
    }

    // ========================================================================
    // NEW SHOOT TELEMETRY - Elastic Dashboard "NewShoot" tab
    // Her 200ms'de (10 loop'ta 1) guncellenir, loop overrun olmaz.
    // RT basili olsa da olmasa da surekli gosterir.
    // ========================================================================
    private int newShootLoopCount = 0;

    public void updateNewShootTelemetry() {
        newShootLoopCount++;
        if (newShootLoopCount % 10 != 0) return;

        // Zone
        double robotX = drivetrain.getState().Pose.getX();
        double dist = vision.getDistanceToOwnHub();
        boolean inOwnZone;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            inOwnZone = alliance.get() == DriverStation.Alliance.Blue
                    ? robotX < 6.5 : robotX > 10.0;
        } else {
            inOwnZone = dist < 7.0;
        }

        SmartDashboard.putBoolean("NewShoot/InOwnZone", inOwnZone);
        SmartDashboard.putNumber("NewShoot/HubDist (m)", Math.round(dist * 100.0) / 100.0);

        // Hood
        SmartDashboard.putNumber("NewShoot/Hood Target", hood.getTargetPosition());
        SmartDashboard.putNumber("NewShoot/Hood Current", hood.getCurrentPosition());
        SmartDashboard.putBoolean("NewShoot/Hood Ready", hood.isPositionWithinTolerance());

        // Shooter
        SmartDashboard.putBoolean("NewShoot/RPM Ready", shooter.isVelocityWithinTolerance());
        SmartDashboard.putNumber("NewShoot/RPM Avg", Math.round(shooter.getAverageMotorRPMAbs()));
        SmartDashboard.putNumber("NewShoot/RPM Target", shooter.getTargetRPM());

        // Aim
        boolean aimed = AlignToHubOdometry.isAimed(drivetrain);
        SmartDashboard.putBoolean("NewShoot/Aimed", aimed);

        // Combined status
        boolean allReady = aimed && shooter.isVelocityWithinTolerance()
                && hood.isPositionWithinTolerance();
        SmartDashboard.putBoolean("NewShoot/ALL READY", allReady);
        SmartDashboard.putString("NewShoot/Status",
                !inOwnZone ? "NEUTRAL ZONE"
                : !aimed ? "AIMING..."
                : !shooter.isVelocityWithinTolerance() ? "SPINUP..."
                : !hood.isPositionWithinTolerance() ? "HOOD..."
                : "FIRE!");
    }

}
