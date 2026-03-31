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
 * TOWER DONUS HIZALAMA - FAZ 1
 * ============================================================================
 *
 * Robot yerinde durur, sadece donerek tower tag'ini ekranin ortasina getirir.
 * Ileri/geri hareket YOKTUR. txnc = 0 olunca settle suresinden sonra biter.
 *
 * KULLANIM (PathPlanner):
 *   path -> alignToTowerRot -> alignToTowerDrive -> climbUp
 * ============================================================================
 */
public class TowerRotateCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxAngularRate;

    // ========================================================================
    // ROTATION PID
    // ========================================================================
    private static final double ROT_KP = 2.0;             // orta guc - surtumeyi yener ama overshoot yapmaz
    private static final double ROT_KD = 0.1;             // hafif frenleme
    private static final double ROT_TOLERANCE = 0.025;    // txnc (~0.8 derece) - kilitlenebilir tolerans
    private static final double ROT_DEADBAND = 0.01;
    private static final double MIN_ROT_OUTPUT = 0.15;    // surtumeyi yenmek icin minimum hiz (rad/s)
    private static final double MAX_ROT_SPEED_SCALE = 0.35; // max hizin %35'i - kontrollü ama yeterli
    private static final double ROT_SLEW_RATE = 2.0;      // yumusak ivmelenme

    // ========================================================================
    // GUVENLIK
    // ========================================================================
    private static final double TIMEOUT_SECONDS = 6.0;    // 6 saniye - rahat hizalansin
    private static final double SETTLE_SECONDS = 0.3;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // Internal state
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(ROT_SLEW_RATE);
    private final Timer timeoutTimer = new Timer();
    private final Timer settleTimer = new Timer();
    private boolean settling = false;
    private double lastTxnc = 0;

    /**
     * @param drivetrain       Swerve drivetrain
     * @param limelightName    Limelight ismi
     * @param maxAngularRate   Maksimum donus hizi (rad/s)
     */
    public TowerRotateCommand(
            CommandSwerveDrivetrain drivetrain,
            String limelightName,
            double maxAngularRate) {

        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.maxAngularRate = maxAngularRate;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
        rotLimiter.reset(0);

        timeoutTimer.stop();
        timeoutTimer.reset();
        timeoutTimer.start();

        settleTimer.stop();
        settleTimer.reset();

        settling = false;
        loopCount = 0;
        lastTxnc = 0;

        SmartDashboard.putString("TowerRot/Status", "Starting...");
        SmartDashboard.putString("TowerRot/AllowedTags", "ALL");
    }

    @Override
    public void execute() {
        loopCount++;
        boolean shouldLog = (loopCount % DASHBOARD_INTERVAL == 0);

        // ================================================================
        // TAG ARAMA
        // ================================================================
        double txnc = 0;
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
                    txnc = best.txnc;
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
            rotLimiter.reset(0);
            settling = false;
            settleTimer.stop();
            settleTimer.reset();
            lastTxnc = 0;
            if (shouldLog) SmartDashboard.putString("TowerRot/Status", "NO TOWER TAG");
            return;
        }

        // ================================================================
        // SADECE DONUS - tag'i ortala
        // ================================================================
        double rotCmd = 0;
        if (Math.abs(txnc) > ROT_DEADBAND) {
            double rotDerivative = txnc - lastTxnc;
            rotCmd = -(ROT_KP * txnc + ROT_KD * rotDerivative);
            if (Math.abs(rotCmd) < MIN_ROT_OUTPUT) {
                rotCmd = Math.copySign(MIN_ROT_OUTPUT, rotCmd);
            }
            double maxRot = maxAngularRate * MAX_ROT_SPEED_SCALE;
            rotCmd = MathUtil.clamp(rotCmd, -maxRot, maxRot);
        }
        lastTxnc = txnc;
        rotCmd = rotLimiter.calculate(rotCmd);

        boolean locked = Math.abs(txnc) < ROT_TOLERANCE;

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
            SmartDashboard.putNumber("TowerRot/TagID", targetTagId);
            SmartDashboard.putNumber("TowerRot/txnc", txnc);
            SmartDashboard.putNumber("TowerRot/RotCmd", rotCmd);
            SmartDashboard.putBoolean("TowerRot/Locked", locked);
            SmartDashboard.putString("TowerRot/Status",
                locked ? "LOCKED Tag " + targetTagId
                       : String.format("Tag %d txnc=%.3f", targetTagId, txnc));
        }

        // Sadece donus, ileri/geri YOK
        drivetrain.setControl(robotCentric
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotCmd));
    }

    @Override
    public boolean isFinished() {
        if (timeoutTimer.hasElapsed(TIMEOUT_SECONDS)) return true;
        if (settling && settleTimer.hasElapsed(SETTLE_SECONDS)) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        drivetrain.setControl(robotCentric
            .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        rotLimiter.reset(0);
        timeoutTimer.stop();
        settleTimer.stop();

        SmartDashboard.putString("TowerRot/Status",
            interrupted ? "Interrupted" : (settling ? "LOCKED!" : "Timeout"));
    }
}
