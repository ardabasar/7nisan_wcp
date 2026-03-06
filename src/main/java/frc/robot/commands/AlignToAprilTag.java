package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * HUB HIZALAMA - Limelight TX Tabanli Dogrudan Hizalama
 * ============================================================================
 *
 * RB basili tut -> Robot Limelight'in gordugu AprilTag'e dogrudan hizalanir.
 * TX degeri = tag'in kamera merkezinden yatay kaymasi (derece).
 * TX = 0 → tag tam merkezde → robot hizali.
 *
 * Bu yontem odometry'ye bagimli DEGILDIR:
 *   - Odometry bozuk olsa bile calisir
 *   - Dogrudan kamera verisini kullanir (gercek zamanli)
 *   - Tag gorunmedigi anda durur (sallanma olmaz)
 *
 * Avantajlari:
 *   - Odometry hatasina duyarsiz
 *   - Anlik ve dogru hizalama
 *   - Basit ve guvenilir
 * ============================================================================
 */
public class AlignToAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxAngularRate;

    // TX-tabanli P + D kontrol
    // TX derece cinsinden, cikis rad/s cinsinden
    private static final double kP = 0.06;
    private static final double kD = 0.004;
    private static final double TOLERANCE_DEG = 2.0;

    private double lastTx = 0;

    // FieldCentric request - sadece rotation kullanacagiz
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Constructor (VisionSubsystem ile - RobotContainer uyumlulugu)
     */
    public AlignToAprilTag(CommandSwerveDrivetrain drivetrain,
                           VisionSubsystem vision,
                           double maxSpeed,
                           double maxAngularRate) {
        this(drivetrain, "limelight", maxSpeed, maxAngularRate);
    }

    /**
     * Constructor (limelight ismi ile)
     */
    public AlignToAprilTag(CommandSwerveDrivetrain drivetrain,
                           String limelightName,
                           double maxSpeed,
                           double maxAngularRate) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.maxAngularRate = maxAngularRate;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        lastTx = 0;
        SmartDashboard.putString("Align/Status", "Starting...");
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            // Tag gorunmuyor - dur, sallanma
            drivetrain.setControl(fieldCentric
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            SmartDashboard.putString("Align/Status", "NO TARGET");
            SmartDashboard.putBoolean("Align/Aimed", false);
            lastTx = 0;
            return;
        }

        // TX: tag'in kamera merkezinden yatay sapma (derece)
        // TX > 0 → tag saga kayik → saga donmemiz lazim (negatif rotasyon)
        double tx = LimelightHelpers.getTX(limelightName);

        // P + D kontrol
        double derivative = tx - lastTx;
        double rotationRate = -(kP * tx + kD * derivative);
        lastTx = tx;

        // Hiz siniri (%60 max hiz - hassas hizalama icin)
        double maxRot = maxAngularRate * 0.6;
        rotationRate = Math.max(-maxRot, Math.min(maxRot, rotationRate));

        // Tolerans icindeyse kilitlen
        boolean aimed = Math.abs(tx) < TOLERANCE_DEG;

        // Uygula
        drivetrain.setControl(fieldCentric
            .withVelocityX(0)
            .withVelocityY(0)
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
