package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * ============================================================================
 * TOWER DUZ ILERLEME - FAZ 2
 * ============================================================================
 *
 * Robot DUZ ILERLER, donus YOKTUR.
 * Tag'e olan mesafeyi olcerek belirtilen mesafeye ulasinca biter.
 * TowerRotateCommand'dan SONRA calistirilmali (tag zaten ortalanmis olmali).
 *
 * KULLANIM (PathPlanner):
 *   path -> alignToTowerRot -> alignToTowerDrive -> climbUp
 * ============================================================================
 */
public class TowerDriveCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxSpeed;
    private final double desiredDistanceMeters;

    // ========================================================================
    // MESAFE PID
    // ========================================================================
    private static final double DIST_KP = 0.9;
    private static final double DIST_KD = 0.06;
    private static final double DIST_TOLERANCE_M = 0.06;
    private static final double DIST_DEADBAND_M = 0.04;
    private static final double MIN_DIST_OUTPUT = 0.06;
    private static final double MAX_APPROACH_SPEED_SCALE = 0.35;
    private static final double X_SLEW_RATE = 1.5;

    // ========================================================================
    // GUVENLIK
    // ========================================================================
    private static final double TIMEOUT_SECONDS = 3.0;
    private static final double SETTLE_SECONDS = 0.25;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // Internal state
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(X_SLEW_RATE);
    private final Timer timeoutTimer = new Timer();
    private final Timer settleTimer = new Timer();
    private boolean settling = false;
    private double lastKnownDistance;
    private double lastDistError = 0;

    /**
     * @param drivetrain              Swerve drivetrain
     * @param limelightName           Limelight ismi
     * @param maxSpeed                Maksimum translasyon hizi (m/s)
     * @param desiredDistanceMeters   Tag'e hedef mesafe (metre)
     */
    public TowerDriveCommand(
            CommandSwerveDrivetrain drivetrain,
            String limelightName,
            double maxSpeed,
            double desiredDistanceMeters) {

        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.maxSpeed = maxSpeed;
        this.desiredDistanceMeters = MathUtil.clamp(desiredDistanceMeters, 0.2, 3.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xLimiter.reset(0);

        timeoutTimer.stop();
        timeoutTimer.reset();
        timeoutTimer.start();

        settleTimer.stop();
        settleTimer.reset();

        settling = false;
        loopCount = 0;
        lastKnownDistance = desiredDistanceMeters;
        lastDistError = 0;

        SmartDashboard.putString("TowerDrive/Status", "Starting...");
    }

    @Override
    public void execute() {
        loopCount++;
        boolean shouldLog = (loopCount % DASHBOARD_INTERVAL == 0);

        // ================================================================
        // TAG ARAMA
        // ================================================================
        double measuredDistance = lastKnownDistance;
        boolean hasTarget = false;
        int targetTagId = -1;

        try {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
            if (fiducials != null && fiducials.length > 0) {
                RawFiducial best = null;
                double bestDist = Double.MAX_VALUE;

                for (RawFiducial fid : fiducials) {
                    if (fid.distToCamera < bestDist) {
                        bestDist = fid.distToCamera;
                        best = fid;
                    }
                }

                if (best != null) {
                    measuredDistance = MathUtil.clamp(best.distToCamera, 0.1, 5.0);
                    lastKnownDistance = measuredDistance;
                    hasTarget = true;
                    targetTagId = best.id;
                }
            }
        } catch (Exception e) {
            hasTarget = false;
        }

        // ================================================================
        // TAG YOK - Dur
        // ================================================================
        if (!hasTarget) {
            drivetrain.setControl(robotCentric
                .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            xLimiter.reset(0);
            settling = false;
            settleTimer.stop();
            settleTimer.reset();
            lastDistError = 0;
            if (shouldLog) SmartDashboard.putString("TowerDrive/Status", "NO TOWER TAG");
            return;
        }

        // ================================================================
        // SADECE DUZ ILERLEME - mesafeye yaklas
        // ================================================================
        double xCmd = 0;
        double distError = measuredDistance - desiredDistanceMeters;
        if (Math.abs(distError) > DIST_DEADBAND_M) {
            double distDerivative = distError - lastDistError;
            double distOutput = DIST_KP * distError + DIST_KD * distDerivative;
            xCmd = distOutput * maxSpeed * MAX_APPROACH_SPEED_SCALE;
            if (Math.abs(xCmd) < MIN_DIST_OUTPUT) {
                xCmd = Math.copySign(MIN_DIST_OUTPUT, xCmd);
            }
            double maxApproach = maxSpeed * MAX_APPROACH_SPEED_SCALE;
            xCmd = MathUtil.clamp(xCmd, -maxApproach, maxApproach);
        }
        lastDistError = distError;
        xCmd = xLimiter.calculate(xCmd);

        boolean locked = Math.abs(distError) < DIST_TOLERANCE_M;

        // ================================================================
        // SETTLE KONTROLU
        // ================================================================
        if (locked) {
            if (!settling) {
                settling = true;
                settleTimer.reset();
                settleTimer.start();
            }
        } else {
            settling = false;
            settleTimer.stop();
            settleTimer.reset();
        }

        if (shouldLog) {
            SmartDashboard.putNumber("TowerDrive/TagID", targetTagId);
            SmartDashboard.putNumber("TowerDrive/MeasuredDist", measuredDistance);
            SmartDashboard.putNumber("TowerDrive/DistError", distError);
            SmartDashboard.putNumber("TowerDrive/XCmd", xCmd);
            SmartDashboard.putBoolean("TowerDrive/Locked", locked);
            SmartDashboard.putString("TowerDrive/Status",
                locked ? "LOCKED Tag " + targetTagId
                       : String.format("Tag %d err=%.2fm", targetTagId, distError));
        }

        // Sadece ileri/geri, donus YOK
        drivetrain.setControl(robotCentric
            .withVelocityX(xCmd)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        if (timeoutTimer.hasElapsed(TIMEOUT_SECONDS)) return true;
        if (settling && settleTimer.hasElapsed(SETTLE_SECONDS)) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(robotCentric
            .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        xLimiter.reset(0);
        timeoutTimer.stop();
        settleTimer.stop();

        SmartDashboard.putString("TowerDrive/Status",
            interrupted ? "Interrupted" : (settling ? "LOCKED!" : "Timeout"));
    }
}
