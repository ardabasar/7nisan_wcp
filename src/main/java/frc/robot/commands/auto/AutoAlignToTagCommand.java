package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * ============================================================================
 * AUTO ALIGN TO TAG COMMAND — Duzeltilmis
 * ============================================================================
 * Oturum basinda (initialize) her sey sifirlanir, ikinci enable'da guvenle
 * yeniden calisir.
 *
 * [FIX] "No robot code" / crash duzeltmeleri:
 *   1) SmartDashboard TAMAMEN THROTTLED — execute'da 0 string allocation
 *      Her loop'ta String.format + putString → GC pause → Watchdog timeout.
 *      Simdi sadece DASHBOARD_INTERVAL loop'ta bir yaziliyor (5Hz).
 *   2) String.format hot path'ten cikarildi — sabit string literalleri kullanildi
 *      isFinished()'de string allocation yok, execute hot path'te minimum.
 *   3) Mesafe hesabi duzeltildi:
 *      cameraPose[2] (sadece Z) → sqrt(x²+z²) (zemin mesafesi, kamera acisi toleranli)
 *   4) settleTimer initialize'da reset VE stop edilir, ikinci calismada temiz baslar
 *   5) Timer'lar end()'de durduruluyor (zaten vardi) + initialize'da tam sifir
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

    // PID ayarlari
    private static final double ROT_KP = 0.035;
    private static final double ROT_KI = 0.0;
    private static final double ROT_KD = 0.003;

    private static final double DIST_KP = 0.8;
    private static final double DIST_KI = 0.0;
    private static final double DIST_KD = 0.05;

    // Toleranslar
    private static final double ROT_TOLERANCE_DEG = 2.0;
    private static final double DIST_TOLERANCE_M  = 0.08;

    // Deadband
    private static final double ROT_DEADBAND_DEG = 0.8;
    private static final double DIST_DEADBAND_M  = 0.05;

    // Minimum cikislar
    private static final double MIN_ROT_OUTPUT  = 0.05;
    private static final double MIN_DIST_OUTPUT = 0.06;

    private static final double ROT_INVERT = 1.0;

    // Hiz limitleri
    private static final double MAX_X_ACCEL   = 1.0;
    private static final double MAX_ROT_ACCEL = 3.0;

    // Otonom guvenlik
    private static final double TIMEOUT_SECONDS = 5.0;
    private static final double SETTLE_SECONDS  = 0.3;

    // [FIX] SmartDashboard throttle — her execute'da putString/putNumber crash yapar
    // 50Hz x 10 = 5Hz guncelleme (Elastic icin yeterli)
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    private final PIDController rotPid;
    private final PIDController distPid;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(MAX_X_ACCEL);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(MAX_ROT_ACCEL);

    private final Timer timeoutTimer = new Timer();
    private final Timer settleTimer  = new Timer();
    private boolean settling = false;

    // Son bilinen mesafe (tag kaybolursa koru)
    private double lastKnownDistance;

    public AutoAlignToTagCommand(
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

        rotPid = new PIDController(ROT_KP, ROT_KI, ROT_KD);
        rotPid.setSetpoint(0);
        rotPid.setTolerance(ROT_TOLERANCE_DEG);
        rotPid.enableContinuousInput(-180, 180);

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

        // [FIX] Her iki timer tam sifirlanir ve DURDURULUR
        //       Onceki calismanin kalinti durumu temizlenir
        //       Bu "no robot code" crash'in ana sebebiydi:
        //       Timer state'i end()'den sonra bozuk kalabiliyordu
        timeoutTimer.stop();
        timeoutTimer.reset();
        timeoutTimer.start();

        settleTimer.stop();
        settleTimer.reset();
        // settleTimer burada START edilmez — sadece bothLocked olunca baslar

        settling         = false;
        loopCount        = 0;
        lastKnownDistance = desiredDistanceMeters;

        SmartDashboard.putString("AutoAlign/Status", "Aligning...");
        SmartDashboard.putNumber("AutoAlign/TargetDist", desiredDistanceMeters);
    }

    @Override
    public void execute() {
        loopCount++;
        boolean shouldLog = (loopCount % DASHBOARD_INTERVAL == 0);

        boolean hasTarget = LimelightHelpers.getTV(limelightName);

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
            if (shouldLog) SmartDashboard.putString("AutoAlign/Status", "No Tag");
            return;
        }

        double tx = LimelightHelpers.getTX(limelightName);

        // ====================================================================
        // MESAFE OLCUMU
        // [FIX] cameraPose[2] (sadece Z ekseni) yerine sqrt(x^2 + z^2) kullanilir.
        //
        // Neden? Kamera robota egimli monte edilirse (tipik FRC kurulumu):
        //   - cameraPose[2] = Z ekseni, tag'e kamera optical axis mesafesi
        //   - cameraPose[0] = X ekseni, yatay kayma
        //   - Y'yi (dikey) yoksayarak zemin projeksiyonu alinir
        //   - Bu robot-tag arasindaki GERCEK yatay mesafeye yakin deger verir
        //
        // Ornek: 1.5m mesafede kamera 10° egik → cameraPose[2]=1.477m ama
        //        gercek yatay = sqrt(1.477² + 0.26²) ≈ 1.50m (duzeltilmis)
        // ====================================================================
        double measuredDistance;
        double[] cameraPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
        if (cameraPose != null && cameraPose.length >= 3 && cameraPose[2] > 0.05) {
            // XZ duzleminde Oklid mesafesi (zemin projeksiyonu)
            double cx = cameraPose[0];
            double cz = cameraPose[2];
            measuredDistance = Math.sqrt(cx * cx + cz * cz);
            measuredDistance = MathUtil.clamp(measuredDistance, 0.1, 6.0);
            lastKnownDistance = measuredDistance;
        } else {
            // Yedek: ta'dan tahmin
            double ta = LimelightHelpers.getTA(limelightName);
            measuredDistance = estimateDistanceFromArea(ta);
        }

        // ====================================================================
        // DONUS KONTROLU
        // ====================================================================
        double rotCmd = 0;
        if (Math.abs(tx) > ROT_DEADBAND_DEG) {
            double rotPidOutput = rotPid.calculate(tx);
            rotCmd = rotPidOutput * maxAngularRate * rotScale * ROT_INVERT;
            if (Math.abs(rotCmd) < MIN_ROT_OUTPUT) {
                rotCmd = Math.copySign(MIN_ROT_OUTPUT, rotCmd);
            }
            rotCmd = MathUtil.clamp(rotCmd, -maxAngularRate * rotScale, maxAngularRate * rotScale);
        }
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
        // KILIT KONTROLU (settle timer)
        // ====================================================================
        boolean rotLocked  = Math.abs(tx)        < ROT_TOLERANCE_DEG;
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

        // [FIX] SmartDashboard throttle — sadece 5Hz
        //       Onceki kod her loop'ta putString + String.format yapiyordu
        //       Bu GC pressure → Watchdog timeout → "No robot code"
        if (shouldLog) {
            SmartDashboard.putBoolean("AutoAlign/HasTarget", true);
            SmartDashboard.putNumber("AutoAlign/TX",          tx);
            SmartDashboard.putNumber("AutoAlign/MeasuredDist", measuredDistance);
            SmartDashboard.putNumber("AutoAlign/DistError",   distError);
            SmartDashboard.putNumber("AutoAlign/RotCmd",      rotCmd);
            SmartDashboard.putNumber("AutoAlign/XCmd",        xCmd);
            SmartDashboard.putNumber("AutoAlign/Elapsed",     timeoutTimer.get());

            if (bothLocked) {
                SmartDashboard.putString("AutoAlign/Status", "Settling");
            } else if (rotLocked) {
                SmartDashboard.putString("AutoAlign/Status", "Dist Only");
            } else if (distLocked) {
                SmartDashboard.putString("AutoAlign/Status", "Rot Only");
            } else {
                SmartDashboard.putString("AutoAlign/Status", "Aligning");
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
        // [FIX] isFinished'de SmartDashboard yazimi YOK — her loop calisir, string allocation olmaz
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

    /**
     * Tag alanindan (ta) mesafeyi tahmin eder — yedek yontem.
     * Kalibrasyona gore ayarlanabilir.
     */
    private double estimateDistanceFromArea(double ta) {
        if (ta < 0.1) return lastKnownDistance; // son bilinen mesafeyi koru
        return MathUtil.clamp(25.0 / Math.sqrt(ta), 0.3, 5.0);
    }
}