package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * ODOMETRY BAZLI HUB HIZALAMA - 614 TARZI
 * ============================================================================
 *
 * Robot odometry (pose) kullanarak hub merkezine dogru yonelir.
 * atan2 ile hedef heading hesaplanir, PID ile rotation kontrol edilir.
 *
 * AVANTAJLARI:
 *   - AprilTag gorunmese bile calisir (odometry yeterli)
 *   - Vision seed yapildiktan sonra surekli aktif
 *   - Surus sirasinda calisir (surucu X/Y kontrolu aktif kalir)
 *
 * ALLIANCE FILTRELEME:
 *   - Blue Alliance: Blue Hub merkezine yonelir
 *   - Red Alliance: Red Hub merkezine yonelir
 *   - Alliance bilinmiyorsa en yakin hub'a yonelir
 *
 * REFERANS: Team 614 - rotateToAllianceTagWhileDriving()
 * ============================================================================
 */
public class AlignToHubOdometry extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier velocityXSupplier;
    private final DoubleSupplier velocityYSupplier;

    // ========================================================================
    // PD KONTROL PARAMETRELERI (614 referansli, 9545'e uyarlanmis)
    // ========================================================================
    private static final double ROTATION_KP = 4.5;                     // P: ana duzeltme gucu
    private static final double ROTATION_KD = 0.15;                    // D: titresimi onler (damping)
    private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 6.0;
    private static final double MIN_ANGULAR_SPEED_RAD_PER_SEC = 0.3;
    private static final double ANGLE_TOLERANCE_DEGREES = 0.8;         // 0.8° altinda → dur, titreme
    private static final double ANGLE_DEADBAND_DEGREES = 0.3;          // 0.3° altinda → omega=0

    // Strafing compensation - surus sirasinda hedefe kilitli kalmayi saglar
    private static final double STRAFE_COMPENSATION_FACTOR = 0.75;

    private double lastHeadingError = 0.0;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Teleop constructor - surucu X/Y kontrolu + otomatik hub hizalama
     */
    private final VisionSubsystem vision;

    public AlignToHubOdometry(CommandSwerveDrivetrain drivetrain,
                               VisionSubsystem vision,
                               DoubleSupplier velocityX,
                               DoubleSupplier velocityY) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.velocityXSupplier = velocityX;
        this.velocityYSupplier = velocityY;
        addRequirements(drivetrain);
    }

    /**
     * Otonom constructor - sadece rotation (surus yok)
     */
    public AlignToHubOdometry(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this(drivetrain, vision, () -> 0, () -> 0);
    }

    @Override
    public void initialize() {
        vision.setAlignmentActive(true);
        lastHeadingError = 0.0;
        SmartDashboard.putString("HubAlign/Status", "Starting...");
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        // Alliance'a gore hub merkezi sec
        Translation2d hubCenter = getHubCenter(robotPos);

        // Robot -> Hub vektoru
        Translation2d toHub = hubCenter.minus(robotPos);

        // Hedef heading: atan2 ile hub'a bakan aci
        Rotation2d desiredHeading = new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));

        // Heading error (normalize -pi ile +pi arasi)
        double headingError = desiredHeading.minus(robotPose.getRotation()).getRadians();
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        // Surucu girdileri
        double vx = velocityXSupplier.getAsDouble();
        double vy = velocityYSupplier.getAsDouble();

        // Strafing compensation - yana hareket ederken hedefe kilitli kal
        double strafeComp = 0.0;
        if (Math.abs(vx) > 0.01 || Math.abs(vy) > 0.01) {
            strafeComp = vy * STRAFE_COMPENSATION_FACTOR;
        }

        // PD kontrol: P ana guc, D titresimi onler (frenleme etkisi)
        double derivative = headingError - lastHeadingError;
        lastHeadingError = headingError;

        double omega = (ROTATION_KP * headingError) + (ROTATION_KD * derivative * 50.0) + strafeComp;
        // 50.0 = ~20ms loop'a gore D olcekleme

        double headingErrorDeg = Math.abs(Math.toDegrees(headingError));

        // Deadband: cok kucuk hatada omega=0 → titresimi tamamen keser
        if (headingErrorDeg < ANGLE_DEADBAND_DEGREES) {
            omega = 0.0;
        }
        // Minimum snap: deadband ustu ama cok kucuk omega → en az MIN ile don
        else if (headingErrorDeg > ANGLE_TOLERANCE_DEGREES
                && Math.abs(omega) < MIN_ANGULAR_SPEED_RAD_PER_SEC) {
            omega = Math.copySign(MIN_ANGULAR_SPEED_RAD_PER_SEC, omega);
        }

        // Max hiz siniri
        omega = Math.max(-MAX_ANGULAR_SPEED_RAD_PER_SEC,
                Math.min(MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

        // Tolerans kontrolu
        boolean aimed = headingErrorDeg < ANGLE_TOLERANCE_DEGREES;

        // Swerve kontrolu uygula
        drivetrain.setControl(fieldCentric
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omega));

        // Dashboard
        SmartDashboard.putNumber("HubAlign/HeadingError", Math.toDegrees(headingError));
        SmartDashboard.putNumber("HubAlign/Omega", omega);
        SmartDashboard.putNumber("HubAlign/HubDist", toHub.getNorm());
        SmartDashboard.putBoolean("HubAlign/Aimed", aimed);
        SmartDashboard.putString("HubAlign/Status",
            aimed ? "LOCKED!"
                  : String.format("Error: %.1f°", Math.toDegrees(headingError)));
    }

    @Override
    public void end(boolean interrupted) {
        vision.setAlignmentActive(false);
        drivetrain.setControl(fieldCentric
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        SmartDashboard.putString("HubAlign/Status", interrupted ? "Cancelled" : "Done");
    }

    @Override
    public boolean isFinished() {
        return false; // RB basili tutuldugu surece calisir
    }

    /**
     * Alliance'a gore hub merkezi secer.
     * Alliance bilinmiyorsa en yakin hub'i dondurur.
     */
    private Translation2d getHubCenter(Translation2d robotPos) {
        Translation2d redHub = VisionSubsystem.getRedHubCenter();
        Translation2d blueHub = VisionSubsystem.getBlueHubCenter();

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red ? redHub : blueHub;
        }

        // Alliance bilinmiyorsa en yakin hub
        double distRed = robotPos.getDistance(redHub);
        double distBlue = robotPos.getDistance(blueHub);
        return distRed < distBlue ? redHub : blueHub;
    }

    /**
     * Robot hedef acida mi? (ShootCommand gibi disaridan kontrol icin)
     */
    public static boolean isAimed(CommandSwerveDrivetrain drivetrain) {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d redHub = VisionSubsystem.getRedHubCenter();
        Translation2d blueHub = VisionSubsystem.getBlueHubCenter();

        Translation2d hubCenter;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            hubCenter = alliance.get() == DriverStation.Alliance.Red ? redHub : blueHub;
        } else {
            hubCenter = robotPos.getDistance(redHub) < robotPos.getDistance(blueHub) ? redHub : blueHub;
        }

        Translation2d toHub = hubCenter.minus(robotPos);
        Rotation2d desiredHeading = new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));
        double headingError = desiredHeading.minus(robotPose.getRotation()).getDegrees();
        return Math.abs(headingError) < 5.0;
    }
}
