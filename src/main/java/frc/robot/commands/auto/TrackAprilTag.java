package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * ============================================================================
 * TRACK APRIL TAG — Surekli AprilTag Takip, Duzeltilmis
 * ============================================================================
 * [FIX] "No robot code" / crash duzeltmeleri:
 *   1) SmartDashboard throttle — execute'da sadece 5Hz guncelleme
 *      String.format hot path'ten tamamen cikarildi
 *   2) rotationOnly boolean flag eklendi:
 *      Onceki -1.0 sentinel degeri MathUtil.clamp(0.3, 5.0)'a donusuyordu
 *      → distanceMode = (0.3 > 0) = TRUE → istemeden mesafe modu aciliyordu!
 *      Simdi rotationOnly=true → mesafe kontrolu tamamen devre disi
 *   3) Mesafe hesabi duzeltildi:
 *      cameraPose[2] → sqrt(x^2+z^2) zemin projeksiyonu
 *   4) initialize()'da tum state tam sifirlanir (ikinci enable guvenli)
 *
 * KULLANIM:
 *   // Mesafe + donus
 *   joystick.leftTrigger(0.5).whileTrue(
 *       new TrackAprilTag(drivetrain, "limelight", maxSpeed, maxRate, 1.0, 0.5, 0.6));
 *
 *   // Sadece donus (mesafe kontrolu yok)
 *   joystick.rightBumper().whileTrue(
 *       new TrackAprilTag(drivetrain, "limelight", maxSpeed, maxRate));
 * ============================================================================
 */
public class TrackAprilTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final double desiredDistanceMeters;

    // [FIX] rotationOnly flag — eski -1.0 sentinel kaldirild
    //       Boylece distanceMode mantigi net ve guvenli
    private final boolean rotationOnly;

    // PID ayarlari (orijinal tune edilmis degerler)
    private static final double ROT_KP  = 0.035;
    private static final double ROT_KI  = 0.0;
    private static final double ROT_KD  = 0.003;

    private static final double DIST_KP = 0.8;
    private static final double DIST_KI = 0.0;
    private static final double DIST_KD = 0.05;

    private static final double ROT_DEADBAND_DEG = 1.0;
    private static final double DIST_DEADBAND_M  = 0.06;

    private static final double MIN_ROT_OUTPUT  = 0.05;
    private static final double MIN_DIST_OUTPUT = 0.05;

    private final double xScale;
    private final double rotScale;

    private static final double MAX_X_ACCEL   = 1.2;
    private static final double MAX_ROT_ACCEL = 3.5;

    // [FIX] SmartDashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    private final PIDController rotPid;
    private final PIDController distPid;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(MAX_X_ACCEL);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(MAX_ROT_ACCEL);

    private double lastKnownDistance;
    private boolean hadTarget = false;

    /**
     * Tam constructor: mesafe + donus takibi
     *
     * @param desiredDistanceMeters Hedef mesafe (metre), ornek 1.0
     * @param xScale                Ileri/geri hiz olcegi (0.0-1.0)
     * @param rotScale              Donus hiz olcegi (0.0-1.0)
     */
    public TrackAprilTag(
            CommandSwerveDrivetrain drivetrain,
            String limelightName,
            double maxSpeed,
            double maxAngularRate,
            double desiredDistanceMeters,
            double xScale,
            double rotScale) {

        this.drivetrain            = drivetrain;
        this.limelightName         = limelightName;
        this.maxSpeed              = maxSpeed;
        this.maxAngularRate        = maxAngularRate;
        this.desiredDistanceMeters = MathUtil.clamp(desiredDistanceMeters, 0.3, 5.0);
        this.xScale                = MathUtil.clamp(xScale,   0.0, 1.0);
        this.rotScale              = MathUtil.clamp(rotScale, 0.0, 1.0);
        this.rotationOnly          = false; // mesafe modu ACIK

        rotPid = new PIDController(ROT_KP, ROT_KI, ROT_KD);
        rotPid.setSetpoint(0);
        rotPid.enableContinuousInput(-180, 180);

        distPid = new PIDController(DIST_KP, DIST_KI, DIST_KD);
        distPid.setSetpoint(this.desiredDistanceMeters);

        addRequirements(drivetrain);
    }

    /**
     * Sadece donus constructor — mesafe kontrolu yok, robot kalmaz.
     *
     * [FIX] Onceki: this(..., -1.0, 0.0, 0.6) → clamp → 0.3m hedef → mesafe modu ACILIYORDU!
     *       Simdi rotationOnly=true ile mesafe modu kesinlikle kapali.
     */
    public TrackAprilTag(
            CommandSwerveDrivetrain drivetrain,
            String limelightName,
            double maxSpeed,
            double maxAngularRate) {

        this.drivetrain            = drivetrain;
        this.limelightName         = limelightName;
        this.maxSpeed              = maxSpeed;
        this.maxAngularRate        = maxAngularRate;
        this.desiredDistanceMeters = 0.0; // kullanilmayacak
        this.xScale                = 0.0;
        this.rotScale              = 0.6;
        this.rotationOnly          = true; // mesafe modu KAPALI

        rotPid = new PIDController(ROT_KP, ROT_KI, ROT_KD);
        rotPid.setSetpoint(0);
        rotPid.enableContinuousInput(-180, 180);

        // distPid rotationOnly modda kullanilmaz ama NPE onlemek icin olustur
        distPid = new PIDController(DIST_KP, DIST_KI, DIST_KD);
        distPid.setSetpoint(0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);

        rotPid.reset();
        distPid.reset();
        xLimiter.reset(0);
        rotLimiter.reset(0);

        loopCount        = 0;
        hadTarget        = false;
        lastKnownDistance = rotationOnly ? 2.0 : desiredDistanceMeters;

        SmartDashboard.putString("Track/Status", "Tracking");
    }

    @Override
    public void execute() {
        loopCount++;
        boolean shouldLog = (loopCount % DASHBOARD_INTERVAL == 0);

        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        // ====================================================================
        // TAG YOK — yumusak dur
        // ====================================================================
        if (!hasTarget) {
            if (hadTarget) {
                // Tag yeni kayboldu — SlewRateLimiter ile yumusak sifirla
                double rotCmd = rotLimiter.calculate(0);
                double xCmd   = xLimiter.calculate(0);
                drivetrain.setControl(
                    robotCentric.withVelocityX(xCmd).withVelocityY(0).withRotationalRate(rotCmd)
                );
            } else {
                drivetrain.setControl(
                    robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
                );
            }
            hadTarget = false;
            if (shouldLog) SmartDashboard.putString("Track/Status", "No Tag");
            return;
        }

        hadTarget = true;
        double tx = LimelightHelpers.getTX(limelightName);

        // ====================================================================
        // MESAFE OLCUMU
        // [FIX] cameraPose[2] yerine sqrt(x^2+z^2) — zemin projeksiyonu
        //       Egik kamera kurulumunda gercek yatay mesafeyi verir
        // ====================================================================
        double measuredDistance = lastKnownDistance;

        if (!rotationOnly) {
            double[] cameraPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
            if (cameraPose != null && cameraPose.length >= 3 && cameraPose[2] > 0.05) {
                double cx = cameraPose[0];
                double cz = cameraPose[2];
                measuredDistance = MathUtil.clamp(
                    Math.sqrt(cx * cx + cz * cz), 0.1, 6.0
                );
                lastKnownDistance = measuredDistance;
            } else {
                double ta = LimelightHelpers.getTA(limelightName);
                if (ta > 0.1) {
                    measuredDistance = MathUtil.clamp(25.0 / Math.sqrt(ta), 0.3, 5.0);
                    lastKnownDistance = measuredDistance;
                }
            }
        }

        // ====================================================================
        // DONUS KONTROLU
        // ====================================================================
        double rotCmd = 0;
        if (Math.abs(tx) > ROT_DEADBAND_DEG) {
            rotCmd = rotPid.calculate(tx) * maxAngularRate * rotScale;
            if (Math.abs(rotCmd) < MIN_ROT_OUTPUT) {
                rotCmd = Math.copySign(MIN_ROT_OUTPUT, rotCmd);
            }
            rotCmd = MathUtil.clamp(rotCmd, -maxAngularRate * rotScale, maxAngularRate * rotScale);
        }
        rotCmd = rotLimiter.calculate(rotCmd);

        // ====================================================================
        // MESAFE KONTROLU (sadece rotationOnly=false ise)
        // ====================================================================
        double xCmd = 0;
        double distError = 0;
        if (!rotationOnly) {
            distError = measuredDistance - desiredDistanceMeters;
            if (Math.abs(distError) > DIST_DEADBAND_M) {
                xCmd = -distPid.calculate(measuredDistance) * maxSpeed * xScale;
                if (Math.abs(xCmd) < MIN_DIST_OUTPUT) {
                    xCmd = Math.copySign(MIN_DIST_OUTPUT, xCmd);
                }
                xCmd = MathUtil.clamp(xCmd, -maxSpeed * xScale, maxSpeed * xScale);
            }
        }
        xCmd = xLimiter.calculate(xCmd);

        drivetrain.setControl(
            robotCentric.withVelocityX(xCmd).withVelocityY(0).withRotationalRate(rotCmd)
        );

        // [FIX] Throttled SmartDashboard — String.format hot path'ten cikarildi
        if (shouldLog) {
            SmartDashboard.putNumber("Track/TX",   tx);
            SmartDashboard.putNumber("Track/Dist", measuredDistance);
            SmartDashboard.putNumber("Track/DistError", distError);

            boolean locked = Math.abs(tx) < 2.0
                && (rotationOnly || Math.abs(distError) < 0.08);
            SmartDashboard.putString("Track/Status", locked ? "LOCKED" : "Tracking");
        }
    }

    @Override
    public boolean isFinished() {
        // Surekli calisir — whileTrue() ile kontrol edilir
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        try {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        } catch (Exception e) {
            // Limelight baglanti hatasi command'i cokertmesin
        }

        drivetrain.setControl(
            robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
        xLimiter.reset(0);
        rotLimiter.reset(0);

        // State'i sifirla — tekrar schedule edilirse temiz baslasin
        hadTarget = false;

        SmartDashboard.putString("Track/Status", interrupted ? "Cancelled" : "Done");
    }
}
