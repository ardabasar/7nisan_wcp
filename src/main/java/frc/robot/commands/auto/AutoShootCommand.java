package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.ShootCommand;

/**
 * OTONOM SHOOT COMMAND
 *
 * ShootCommand ile ayni mantik ama zamana dayali otomatik bitis:
 *   1) Shooter spin-up (hazirlik)
 *   2) Feeder + Hopper baslar (toplar atilir)
 *   3) Besleme baslayinca FEED_DURATION kadar bekle → bitti
 *
 * Toplam sure: spin-up (~0.6s) + feed (2.5s) = ~3s
 * 10s yerine ~3s!
 */
public class AutoShootCommand extends Command {
    // Besleme suresi: toplar atilmaya basladiktan sonra bu kadar calis
    // 30-40 top icin 6s gerekli (~5 top/sn)
    private static final double FEED_DURATION_SECONDS = 6.0;

    // Maksimum toplam sure (guvenlik - takilma onleme)
    private static final double MAX_TOTAL_SECONDS = 7.0;

    private final ShootCommand innerShoot;
    private double commandStartTime = 0;
    private double feedStartTime = 0;
    private boolean feedingDetected = false;

    public AutoShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
            FeederSubsystem feeder, HopperSubsystem hopper,
            VisionSubsystem vision, String limelightName,
            IntakeArmSubsystem intakeArm) {
        // ShootCommand'in tum mantigi aynen kullanilir
        innerShoot = new ShootCommand(shooter, hood, feeder, hopper, vision, limelightName, intakeArm);

        // Ayni requirements
        if (intakeArm != null) {
            addRequirements(shooter, hood, feeder, hopper, intakeArm);
        } else {
            addRequirements(shooter, hood, feeder, hopper);
        }
    }

    public AutoShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
            FeederSubsystem feeder, HopperSubsystem hopper,
            VisionSubsystem vision, String limelightName) {
        this(shooter, hood, feeder, hopper, vision, limelightName, null);
    }

    @Override
    public void initialize() {
        commandStartTime = Timer.getFPGATimestamp();
        feedStartTime = 0;
        feedingDetected = false;
        innerShoot.initialize();
    }

    @Override
    public void execute() {
        innerShoot.execute();

        // Feeder basladigini SmartDashboard'dan takip et
        if (!feedingDetected && SmartDashboard.getBoolean("Shoot/AllowFeed", false)) {
            feedingDetected = true;
            feedStartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        double elapsed = Timer.getFPGATimestamp() - commandStartTime;

        // Guvenlik timeout
        if (elapsed >= MAX_TOTAL_SECONDS) return true;

        // Besleme basladiysa, FEED_DURATION kadar bekle
        if (feedingDetected) {
            double feedElapsed = Timer.getFPGATimestamp() - feedStartTime;
            return feedElapsed >= FEED_DURATION_SECONDS;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        innerShoot.end(interrupted);
    }
}
