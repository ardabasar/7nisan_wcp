package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * HUB HIZALAMA - EN YAKIN TAG'E DOGRUDAN HIZALAMA + SURUS
 * ============================================================================
 *
 * RB basili tut -> Robot en yakin AprilTag'e hizalanir.
 * Surucu ayni anda X/Y hareket yapabilir (joystick aktif kalir).
 * Sadece ROTATION otomatik kontrol edilir.
 *
 * En yakin tag bulma:
 *   - getRawFiducials() ile tum gorulen tagleri al
 *   - distToCamera en kucuk olani sec
 *   - Onun txnc degerini derece'ye cevir ve P+D kontrol uygula
 *
 * TX -> derece donusumu:
 *   txnc * (FOV/2) = yaklasik derece
 *   LL3: FOV ~63.3 -> txnc=1.0 -> ~31.6 derece
 *
 * Odometry'ye BAGIMLI DEGILDIR.
 * ============================================================================
 */
public class AlignToAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxAngularRate;
    private final DoubleSupplier velocityXSupplier;
    private final DoubleSupplier velocityYSupplier;

    // TX-tabanli P + D kontrol
    private static final double kP = 0.06;
    private static final double kD = 0.004;
    private static final double TOLERANCE_DEG = 1.0;

    // Limelight yatay FOV / 2 (derece) - txnc -> derece donusumu icin
    // LL2: ~29.8, LL3: ~31.65
    private static final double LL_HORIZ_HALF_FOV_DEG = 31.65;

    private double lastTx = 0;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Constructor - Teleop icin (surucu X/Y kontrolu + otomatik rotation)
     *
     * @param velocityX surucu ileri/geri hiz (m/s)
     * @param velocityY surucu sag/sol hiz (m/s)
     */
    public AlignToAprilTag(CommandSwerveDrivetrain drivetrain,
                           String limelightName,
                           double maxSpeed,
                           double maxAngularRate,
                           DoubleSupplier velocityX,
                           DoubleSupplier velocityY) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.maxAngularRate = maxAngularRate;
        this.velocityXSupplier = velocityX;
        this.velocityYSupplier = velocityY;

        addRequirements(drivetrain);
    }

    /**
     * Constructor - Otonom icin (surus yok, sadece rotation)
     * VisionSubsystem parametresi geriye uyumluluk icin kalir.
     */
    public AlignToAprilTag(CommandSwerveDrivetrain drivetrain,
                           VisionSubsystem vision,
                           double maxSpeed,
                           double maxAngularRate) {
        this(drivetrain, "limelight", maxSpeed, maxAngularRate, () -> 0, () -> 0);
    }

    @Override
    public void initialize() {
        lastTx = 0;
        SmartDashboard.putString("Align/Status", "Starting...");
    }

    @Override
    public void execute() {
        // En yakin tag'i bul (rawFiducials ile)
        double tx = 0;
        boolean hasTarget = false;

        try {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
            if (fiducials != null && fiducials.length > 0) {
                // distToCamera en kucuk olan = en yakin tag
                RawFiducial nearest = fiducials[0];
                for (int i = 1; i < fiducials.length; i++) {
                    if (fiducials[i].distToCamera < nearest.distToCamera) {
                        nearest = fiducials[i];
                    }
                }
                // txnc -> derece donusumu
                tx = nearest.txnc * LL_HORIZ_HALF_FOV_DEG;
                hasTarget = true;

                SmartDashboard.putNumber("Align/NearestTagID", nearest.id);
                SmartDashboard.putNumber("Align/NearestDist",
                    Math.round(nearest.distToCamera * 100.0) / 100.0);
            }
        } catch (Exception e) {
            // rawFiducials alinamazsa fallback: standart getTX
            hasTarget = LimelightHelpers.getTV(limelightName);
            if (hasTarget) {
                tx = LimelightHelpers.getTX(limelightName);
            }
        }

        // Surucu X/Y girdisi (joystick'ten)
        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();

        if (!hasTarget) {
            // Tag yok - surucu kontrolu gec, otomatik rotation yok
            drivetrain.setControl(fieldCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(0));
            SmartDashboard.putString("Align/Status", "NO TARGET");
            SmartDashboard.putBoolean("Align/Aimed", false);
            lastTx = 0;
            return;
        }

        // P + D kontrol
        double derivative = tx - lastTx;
        double rotationRate = -(kP * tx + kD * derivative);
        lastTx = tx;

        // Hiz siniri (%60 max hiz)
        double maxRot = maxAngularRate * 0.6;
        rotationRate = Math.max(-maxRot, Math.min(maxRot, rotationRate));

        // Tolerans icindeyse kilitlen
        boolean aimed = Math.abs(tx) < TOLERANCE_DEG;

        // Uygula: surucu X/Y + otomatik rotation
        drivetrain.setControl(fieldCentric
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(aimed ? 0 : rotationRate));

        // Dashboard
        SmartDashboard.putNumber("Align/TX", tx);
        SmartDashboard.putNumber("Align/RotRate", rotationRate);
        SmartDashboard.putBoolean("Align/Aimed", aimed);
        SmartDashboard.putString("Align/Status",
            aimed ? "LOCKED!" : String.format("TX: %.1f°", tx));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(fieldCentric
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        SmartDashboard.putString("Align/Status", interrupted ? "Cancelled" : "Done");
    }

    @Override
    public boolean isFinished() {
        return false; // RB basili tutuldugu surece calisir
    }
}
