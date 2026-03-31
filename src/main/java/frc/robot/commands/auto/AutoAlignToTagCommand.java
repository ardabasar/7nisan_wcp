package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * ============================================================================
 * AUTO ALIGN TO TAG COMMAND — HASSAS HIZALAMA
 * ============================================================================
 *
 * HIZALAMA MANTIGI:
 *   - txnc degeri kullanilarak merkez hizalama (hedef = 0)
 *   - Mesafe kontrolu ile istenilen uzakliga git
 *
 * OTONOM KULLANIM:
 *   Commands.sequence(pathCommand, new AutoAlignToTagCommand(...));
 * ============================================================================
 */
public class AutoAlignToTagCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final double desiredDistanceMeters;
    private final double xScale;
    private final double rotScale;

    // PID ayarlari - txnc tabanli (daha hassas)
    private static final double ROT_KP = 4.0;   // txnc * kP = rad/s
    private static final double ROT_KI = 0.0;
    private static final double ROT_KD = 0.12;

    private static final double DIST_KP = 0.8;
    private static final double DIST_KI = 0.0;
    private static final double DIST_KD = 0.05;

    // Toleranslar - txnc tabanli
    private static final double ROT_TOLERANCE = 0.02;   // txnc toleransi (~0.6 derece)
    private static final double DIST_TOLERANCE_M = 0.08;

    // Deadband
    private static final double ROT_DEADBAND = 0.01;    // txnc deadband
    private static final double DIST_DEADBAND_M = 0.05;

    // Minimum cikislar
    private static final double MIN_ROT_OUTPUT = 0.05;
    private static final double MIN_DIST_OUTPUT = 0.06;

    // Hiz limitleri
    private static final double MAX_X_ACCEL = 1.0;
    private static final double MAX_ROT_ACCEL = 3.0;

    // Otonom guvenlik
    private static final double TIMEOUT_SECONDS = 5.0;
    private static final double SETTLE_SECONDS = 0.3;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    private final PIDController rotPid;
    private final PIDController distPid;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_X_ACCEL);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(MAX_ROT_ACCEL);

    private final Timer timeoutTimer = new Timer();
    private final Timer settleTimer = new Timer();
    private boolean settling = false;

    private double lastKnownDistance;
    private double lastTxnc = 0;

    public AutoAlignToTagCommand(
            CommandSwerveDrivetrain drivetrain,
            String limelightName,
            double maxSpeed,
            double maxAngularRate,
            double desiredDistanceMeters,
            double xScale,
            double rotScale) {

        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.desiredDistanceMeters = MathUtil.clamp(desiredDistanceMeters, 0.3, 5.0);
        this.xScale = MathUtil.clamp(xScale, 0.0, 1.0);
        this.rotScale = MathUtil.clamp(rotScale, 0.0, 1.0);

        rotPid = new PIDController(ROT_KP, ROT_KI, ROT_KD);
        rotPid.setSetpoint(0);  // Hedef: txnc = 0 (tag merkezde)
        rotPid.setTolerance(ROT_TOLERANCE);

        distPid = new PIDController(DIST_KP, DIST_KI, DIST_KD);
        distPid.setSetpoint(this.desiredDistanceMeters);
        distPid.setTolerance(DIST_TOLERANCE_M);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);

        rotPid.reset();
        distPid.reset();
        xLimiter.reset(0);
        rotLimiter.reset(0);

        timeoutTimer.stop();
        timeoutTimer.reset();
        timeoutTimer.start();

        settleTimer.stop();
        settleTimer.reset();

        settling = false;
        loopCount = 0;
        lastKnownDistance = desiredDistanceMeters;
        lastTxnc = 0;

        SmartDashboard.putString("AutoAlign/Status", "Aligning...");
        SmartDashboard.putString("AutoAlign/AllowedTags", "ALL");
    }

    @Override
    public void execute() {
        loopCount++;
        boolean shouldLog = (loopCount % DASHBOARD_INTERVAL == 0);

        // En yakin IZIN VERILEN tag'i bul
        double txnc = 0;
        double measuredDistance = lastKnownDistance;
        boolean hasTarget = false;
        int targetTagId = -1;

        try {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
            if (fiducials != null && fiducials.length > 0) {
                RawFiducial nearest = null;
                double nearestDist = Double.MAX_VALUE;

                for (RawFiducial fid : fiducials) {
                    if (fid.distToCamera < nearestDist) {
                        nearestDist = fid.distToCamera;
                        nearest = fid;
                    }
                }

                if (nearest != null) {
                    txnc = nearest.txnc;
                    measuredDistance = nearest.distToCamera;
                    measuredDistance = MathUtil.clamp(measuredDistance, 0.1, 6.0);
                    lastKnownDistance = measuredDistance;
                    hasTarget = true;
                    targetTagId = nearest.id;
                }
            }
        } catch (Exception e) {
            hasTarget = false;
        }

        // Tag yok — dur ve bekle
        if (!hasTarget) {
            drivetrain.setControl(
                robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
            xLimiter.reset(0);
            rotLimiter.reset(0);
            settling = false;
            settleTimer.stop();
            settleTimer.reset();
            lastTxnc = 0;
            if (shouldLog) SmartDashboard.putString("AutoAlign/Status", "No Allowed Tag");
            return;
        }

        // ====================================================================
        // DONUS KONTROLU (txnc tabanli)
        // ====================================================================
        double rotCmd = 0;
        if (Math.abs(txnc) > ROT_DEADBAND) {
            double derivative = txnc - lastTxnc;
            rotCmd = -(ROT_KP * txnc + ROT_KD * derivative);
            if (Math.abs(rotCmd) < MIN_ROT_OUTPUT) {
                rotCmd = Math.copySign(MIN_ROT_OUTPUT, rotCmd);
            }
            rotCmd = MathUtil.clamp(rotCmd, -maxAngularRate * rotScale, maxAngularRate * rotScale);
        }
        lastTxnc = txnc;
        rotCmd = rotLimiter.calculate(rotCmd);

        // ====================================================================
        // MESAFE KONTROLU
        // ====================================================================
        double xCmd = 0;
        double distError = measuredDistance - desiredDistanceMeters;
        if (Math.abs(distError) > DIST_DEADBAND_M) {
            double distPidOutput = distPid.calculate(measuredDistance);
            xCmd = -distPidOutput * maxSpeed * xScale;
            if (Math.abs(xCmd) < MIN_DIST_OUTPUT) {
                xCmd = Math.copySign(MIN_DIST_OUTPUT, xCmd);
            }
            xCmd = MathUtil.clamp(xCmd, -maxSpeed * xScale, maxSpeed * xScale);
        }
        xCmd = xLimiter.calculate(xCmd);

        // ====================================================================
        // KILIT KONTROLU
        // ====================================================================
        boolean rotLocked = Math.abs(txnc) < ROT_TOLERANCE;
        boolean distLocked = Math.abs(distError) < DIST_TOLERANCE_M;
        boolean bothLocked = rotLocked && distLocked;

        if (bothLocked) {
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
            SmartDashboard.putBoolean("AutoAlign/HasTarget", true);
            SmartDashboard.putNumber("AutoAlign/TargetTagID", targetTagId);
            SmartDashboard.putNumber("AutoAlign/txnc", txnc);
            SmartDashboard.putNumber("AutoAlign/MeasuredDist", measuredDistance);
            SmartDashboard.putNumber("AutoAlign/DistError", distError);
            SmartDashboard.putNumber("AutoAlign/RotCmd", rotCmd);
            SmartDashboard.putNumber("AutoAlign/XCmd", xCmd);

            if (bothLocked) {
                SmartDashboard.putString("AutoAlign/Status", "Settling Tag " + targetTagId);
            } else if (rotLocked) {
                SmartDashboard.putString("AutoAlign/Status", "Dist Only Tag " + targetTagId);
            } else if (distLocked) {
                SmartDashboard.putString("AutoAlign/Status", "Rot Only Tag " + targetTagId);
            } else {
                SmartDashboard.putString("AutoAlign/Status", "Aligning Tag " + targetTagId);
            }
        }

        drivetrain.setControl(
            robotCentric
                .withVelocityX(xCmd)
                .withVelocityY(0)
                .withRotationalRate(rotCmd)
        );
    }

    @Override
    public boolean isFinished() {
        if (timeoutTimer.hasElapsed(TIMEOUT_SECONDS)) {
            return true;
        }
        if (settling && settleTimer.hasElapsed(SETTLE_SECONDS)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        drivetrain.setControl(
            robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
        xLimiter.reset(0);
        rotLimiter.reset(0);
        timeoutTimer.stop();
        settleTimer.stop();

        SmartDashboard.putString("AutoAlign/Status",
            interrupted ? "Interrupted" : "Done");
        SmartDashboard.putBoolean("AutoAlign/HasTarget", false);
    }
}
