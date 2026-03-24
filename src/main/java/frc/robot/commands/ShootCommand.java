package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * SHOOT COMMAND - WCP PrepareShotCommand BIREBIR MANTIK
 *
 * RT basili tutuldugunca:
 * 1) Odometry ile hub mesafesi olculur
 * 2) Mesafeye gore InterpolatingTreeMap ile RPM + Hood enterpolasyon
 * 3) Shooter hizlanir (VelocityVoltage PID)
 * 4) Shooter hedef RPM'e ulasinca Feeder (5000 RPM) + Hopper baslar
 * 5) Intake arm zamanlayici ile yukari/asagi sallanir (top agitasyonu)
 * 6) Birakinca hepsi durur, hood default'a doner
 */
public class ShootCommand extends Command {
    private static final double kShooterReadyToleranceRPM = 150.0;
    private static final int kShooterReadyCyclesRequired = 3;
    private static final double kMinFeedDelaySeconds = 0.20;
    private static final double kSpinupFallbackSeconds = 0.6;
    private static final double kSpinupFallbackRatio = 0.85;
    private static final double kDistanceFilterAlpha = 0.10;
    private static final double kHoodDeadband = 0.02;
    private static final Distance kMinShotDistance = Inches.of(36.0);
    private static final Distance kMaxShotDistance = Inches.of(242.0);

    // ========================================================================
    // INTAKE ARM AGITASYON - Zamanlayici tabanli (encoder YOK)
    // ========================================================================
    // Toplari shooter'a itmek icin kol yukari/asagi sallanir.
    // Asagi hiz dusuk: yercekimi yardim eder, motor zorlanmaz.
    private static final double ARM_UP_SPEED = 0.20; // yukari hiz
    private static final double ARM_DOWN_SPEED = -0.20; // asagi hiz (esit kalkip insin)
    private static final double ARM_UP_SECONDS = 0.50; // yukari suresi
    private static final double ARM_DOWN_SECONDS = 0.50; // asagi suresi
    private static final double ARM_CYCLE_SECONDS = ARM_UP_SECONDS + ARM_DOWN_SECONDS;

    // ========================================================================
    // WCP Shot tablosu - InterpolatingTreeMap birebir kopya
    // ========================================================================
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
            (startValue, endValue, q) -> InverseInterpolator.forDouble()
                    .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
            (startValue, endValue, t) -> new Shot(
                    Interpolator.forDouble()
                            .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                    Interpolator.forDouble()
                            .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)));

    static {
        distanceToShotMap.put(Inches.of(36.0), new Shot(2760, 0.12)); // Hub dibinden atis
        distanceToShotMap.put(Inches.of(52.0), new Shot(3220, 0.19)); // En yakin
        distanceToShotMap.put(Inches.of(72.0), new Shot(3393, 0.25)); // Yakin-orta
        distanceToShotMap.put(Inches.of(92.0), new Shot(3565, 0.31)); // Orta-yakin
        distanceToShotMap.put(Inches.of(114.4), new Shot(3766, 0.40)); // Orta (WCP referans)
        distanceToShotMap.put(Inches.of(132.0), new Shot(3910, 0.43)); // Orta-uzak
        distanceToShotMap.put(Inches.of(150.0), new Shot(4083, 0.46)); // Uzak
        distanceToShotMap.put(Inches.of(165.5), new Shot(4198, 0.48)); // WCP uzak referans

        // Uzun menzil (tower/human tarafi yakini) - sahada tune edilecek
        distanceToShotMap.put(Inches.of(182.0), new Shot(4347, 0.51));
        distanceToShotMap.put(Inches.of(200.0), new Shot(4497, 0.55));
        distanceToShotMap.put(Inches.of(220.0), new Shot(4657, 0.59));
        distanceToShotMap.put(Inches.of(235.0), new Shot(4773, 0.62));
        distanceToShotMap.put(Inches.of(242.0), new Shot(4830, 0.64)); // Human side'e yakin ust limit
    }

    // ========================================================================
    // Subsystemler
    // ========================================================================
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final IntakeArmSubsystem intakeArm; // null olabilir
    private final VisionSubsystem vision;
    private final String limelightName;
    private int readyCycles = 0;
    private double spinupStartTimestamp = 0.0;
    private double lastTargetRpm = 0.0;
    private double filteredDistanceMeters = kMinShotDistance.in(Meters);
    private double lastHoodCommand = -1.0;
    private double feedStartTimestamp = 0.0;
    private boolean feedingStarted = false;

    /**
     * Constructor - IntakeArm agitasyonlu (teleop + otonom)
     */
    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
            FeederSubsystem feeder, HopperSubsystem hopper,
            VisionSubsystem vision, String limelightName,
            IntakeArmSubsystem intakeArm) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.hopper = hopper;
        this.vision = vision;
        this.limelightName = limelightName;
        this.intakeArm = intakeArm;

        if (intakeArm != null) {
            addRequirements(shooter, hood, feeder, hopper, intakeArm);
        } else {
            addRequirements(shooter, hood, feeder, hopper);
        }
    }

    /**
     * Constructor - IntakeArm'siz (geriye uyumluluk)
     */
    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
            FeederSubsystem feeder, HopperSubsystem hopper,
            VisionSubsystem vision, String limelightName) {
        this(shooter, hood, feeder, hopper, vision, limelightName, null);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
        readyCycles = 0;
        spinupStartTimestamp = Timer.getFPGATimestamp();
        lastTargetRpm = 0.0;
        filteredDistanceMeters = kMinShotDistance.in(Meters);
        lastHoodCommand = -1.0;
        feedStartTimestamp = 0.0;
        feedingStarted = false;
        feeder.stop();
        hopper.stop();
        if (intakeArm != null) {
            intakeArm.stop();
        }
    }

    @Override
    public void execute() {
        // 1) MESAFE
        Distance rawDistance = getDistanceToHub();
        double clampedDistanceMeters = MathUtil.clamp(
                rawDistance.in(Meters),
                kMinShotDistance.in(Meters),
                kMaxShotDistance.in(Meters));
        filteredDistanceMeters += (clampedDistanceMeters - filteredDistanceMeters) * kDistanceFilterAlpha;
        Distance distanceToHub = Meters.of(filteredDistanceMeters);

        // 2) ENTERPOLASYON
        Shot shot = distanceToShotMap.get(distanceToHub);

        if (Math.abs(shot.shooterRPM - lastTargetRpm) > 125.0) {
            lastTargetRpm = shot.shooterRPM;
            spinupStartTimestamp = Timer.getFPGATimestamp();
            readyCycles = 0;
        }

        // 3) SHOOTER + HOOD
        shooter.setRPM(shot.shooterRPM);

        if (lastHoodCommand < 0 || Math.abs(shot.hoodPosition - lastHoodCommand) > kHoodDeadband) {
            hood.setPosition(shot.hoodPosition);
            lastHoodCommand = shot.hoodPosition;
        }

        // 4) FEEDER + HOPPER
        boolean strictReady = shooter.isVelocityWithinTolerance(kShooterReadyToleranceRPM);
        readyCycles = strictReady ? readyCycles + 1 : 0;
        boolean readyByStrict = readyCycles >= kShooterReadyCyclesRequired;

        double elapsed = Timer.getFPGATimestamp() - spinupStartTimestamp;
        boolean readyByFallback = elapsed >= kSpinupFallbackSeconds
                && shooter.isVelocityAboveRatio(kSpinupFallbackRatio);

        boolean hoodReady = hood.isPositionWithinTolerance();
        boolean allowFeed = elapsed >= kMinFeedDelaySeconds
                && (readyByStrict || readyByFallback)
                && hoodReady;

        if (allowFeed) {
            feeder.feed();
            hopper.run();

            // Feed baslangic zamanini kaydet (arm agitasyonu icin)
            if (!feedingStarted) {
                feedStartTimestamp = Timer.getFPGATimestamp();
                feedingStarted = true;
            }
        } else {
            feeder.stop();
            hopper.stop();
        }

        // 5) INTAKE ARM AGITASYONU (sadece besleme sirasinda)
        if (intakeArm != null) {
            if (feedingStarted) {
                double feedElapsed = Timer.getFPGATimestamp() - feedStartTimestamp;
                double phase = feedElapsed % ARM_CYCLE_SECONDS;

                if (phase < ARM_UP_SECONDS) {
                    intakeArm.setSpeed(ARM_UP_SPEED); // 1s yukari
                } else {
                    intakeArm.setSpeed(ARM_DOWN_SPEED); // 1s asagi (yumusak)
                }
            } else {
                intakeArm.stop();
            }
        }

        // Dashboard
        SmartDashboard.putNumber("Shoot/Distance (inches)", rawDistance.in(Inches));
        SmartDashboard.putNumber("Shoot/Filtered Distance (inches)", distanceToHub.in(Inches));
        SmartDashboard.putNumber("Shoot/Distance (m)", distanceToHub.in(Meters));
        SmartDashboard.putString("Shoot/DistanceSource", lastDistanceSource);
        SmartDashboard.putNumber("Shoot/Target RPM", shot.shooterRPM);
        SmartDashboard.putNumber("Shoot/Hood Pos", shot.hoodPosition);
        SmartDashboard.putBoolean("Shoot/ShooterReady Strict", readyByStrict);
        SmartDashboard.putBoolean("Shoot/ShooterReady Fallback", readyByFallback);
        SmartDashboard.putBoolean("Shoot/HoodReady", hoodReady);
        SmartDashboard.putBoolean("Shoot/AllowFeed", allowFeed);
        SmartDashboard.putNumber("Shoot/SpinupElapsed", elapsed);
        SmartDashboard.putNumber("Shoot/ReadyCycles", readyCycles);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        hopper.stop();
        hood.setDefault();
        if (intakeArm != null) {
            intakeArm.stop();
        }
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);

        SmartDashboard.putBoolean("Shoot/ShooterReady", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ========================================================================
    // MESAFE OLCUMU - Sadece Odometry (hub merkezine mesafe)
    // ========================================================================
    // NEDEN LIMELIGHT DIREKT KULLANILMIYOR:
    // Limelight hangi tag'i goruyorsa ONA mesafe olcer.
    // Eger tower/trench/outpost tag'i goruyorsa yanlis mesafe verir.
    // Odometry her zaman hub MERKEZINE mesafe verir → dogru RPM.
    // ========================================================================
    private String lastDistanceSource = "None";

    private Distance getDistanceToHub() {
        if (vision != null && vision.hasBeenSeeded()) {
            double dist = vision.getDistanceToOwnHub();
            if (dist >= 0 && dist < 20.0) {
                lastDistanceSource = "Odometry";
                return Meters.of(dist);
            }
        }

        lastDistanceSource = "Fallback(1.32m)";
        return Inches.of(52.0);
    }

    // ========================================================================
    // WCP Shot class birebir kopya
    // ========================================================================
    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
