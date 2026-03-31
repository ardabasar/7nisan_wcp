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
 * HUB HIZALAMA - HASSAS MERKEZ HIZALAMA
 * ============================================================================
 *
 * RB basili tut -> Robot izin verilen AprilTag'e hizalanir.
 * Surucu ayni anda X/Y hareket yapabilir (joystick aktif kalir).
 * Sadece ROTATION otomatik kontrol edilir.
 *
 * HIZALAMA MANTIGI:
 *   - getRawFiducials() ile tum gorulen tagleri al
 *   - distToCamera en kucuk olani sec
 *   - txnc degerini kullanarak PID kontrol uygula (txnc = 0 hedef)
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

    // ========================================================================
    // PID KONTROL PARAMETRELERI - HASSAS MERKEZ HIZALAMA
    // ========================================================================
    // txnc degeri -1.0 ile +1.0 arasi normalize edilmis
    // Hedef: txnc = 0 (tag tam merkezde)
    private static final double kP = 4.5;    // Proportional gain (txnc * kP = rad/s)
    private static final double kD = 0.15;   // Derivative gain (titresimi onle)
    private static final double TOLERANCE = 0.02;  // txnc toleransi (0.02 = ~0.6 derece)

    // Limelight yatay FOV / 2 (derece) - sadece dashboard icin
    private static final double LL_HORIZ_HALF_FOV_DEG = 31.65;

    private double lastTxnc = 0;

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
        lastTxnc = 0;
        SmartDashboard.putString("Align/Status", "Starting...");
        SmartDashboard.putString("Align/AllowedTags", "ALL");
    }

    @Override
    public void execute() {
        // En yakin IZIN VERILEN tag'i bul (rawFiducials ile)
        double txnc = 0;
        boolean hasTarget = false;
        int targetTagId = -1;

        try {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
            if (fiducials != null && fiducials.length > 0) {
                // Gorulen taglerden en yakini bul
                RawFiducial nearest = null;
                double nearestDist = Double.MAX_VALUE;

                for (RawFiducial fid : fiducials) {
                    // En yakin olani sec
                    if (fid.distToCamera < nearestDist) {
                        nearestDist = fid.distToCamera;
                        nearest = fid;
                    }
                }

                if (nearest != null) {
                    // txnc dogrudan kullan (normalize -1 ile +1 arasi)
                    // Hedef: txnc = 0 (tag tam merkezde)
                    txnc = nearest.txnc;
                    hasTarget = true;
                    targetTagId = nearest.id;

                    SmartDashboard.putNumber("Align/TargetTagID", nearest.id);
                    SmartDashboard.putNumber("Align/TargetDist",
                        Math.round(nearest.distToCamera * 100.0) / 100.0);
                    SmartDashboard.putNumber("Align/txnc", txnc);
                    // Derece cinsinden goster (dashboard icin)
                    SmartDashboard.putNumber("Align/TX_deg", txnc * LL_HORIZ_HALF_FOV_DEG);
                }
            }
        } catch (Exception e) {
            // Hata durumunda hedef yok
            hasTarget = false;
        }

        // Surucu X/Y girdisi (joystick'ten)
        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();

        if (!hasTarget) {
            // Izin verilen tag yok - surucu kontrolu gec, otomatik rotation yok
            drivetrain.setControl(fieldCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(0));
            SmartDashboard.putString("Align/Status", "NO ALLOWED TAG");
            SmartDashboard.putBoolean("Align/Aimed", false);
            SmartDashboard.putNumber("Align/TargetTagID", -1);
            lastTxnc = 0;
            return;
        }

        // PD kontrol (txnc direkt kullaniliyor, hedef = 0)
        double error = txnc;  // Hedef 0, hata = txnc
        double derivative = txnc - lastTxnc;
        double rotationRate = -(kP * error + kD * derivative);
        lastTxnc = txnc;

        // Hiz siniri (%80 max hiz - daha agresif)
        double maxRot = maxAngularRate * 0.8;
        rotationRate = Math.max(-maxRot, Math.min(maxRot, rotationRate));

        // Tolerans icindeyse kilitlen
        boolean aimed = Math.abs(txnc) < TOLERANCE;

        // Uygula: surucu X/Y + otomatik rotation
        drivetrain.setControl(fieldCentric
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(aimed ? 0 : rotationRate));

        // Dashboard
        SmartDashboard.putNumber("Align/RotRate", rotationRate);
        SmartDashboard.putBoolean("Align/Aimed", aimed);
        SmartDashboard.putString("Align/Status",
            aimed ? "LOCKED on Tag " + targetTagId + "!"
                  : String.format("Tag %d: txnc=%.3f", targetTagId, txnc));
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
