package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * VISION AUTO SEED COMMAND
 * ============================================================================
 * Autonomous baslangicinda robotun GERCEK saha pozisyonunu tespit eder.
 * 
 * PROBLEM:
 *   PathPlanner, auto basinda yolun baslangic noktasini (ornegin x=5, y=4)
 *   odometry'e yazar. Robot fiziksel olarak baska yerdeyse (ornegin x=1, y=2),
 *   odometry yanlis olur ve robot yanlis yere gider.
 * 
 * COZUM:
 *   Bu komut auto'nun EN BASINA eklenir. Limelight ile AprilTag'leri gorup
 *   robotun gercek saha koordinatini tespit eder ve odometry'e yazar.
 *   Boylece PathPlanner robotun GERCEK konumunu bilir.
 * 
 * KULLANIM (RobotContainer'da):
 *   return Commands.sequence(
 *       new VisionAutoSeedCommand(drivetrain, vision, "limelight"),
 *       autoChooser.getSelected()
 *   );
 * 
 * MANTIK:
 *   1) Limelight LED'leri ac, vision'i enable et
 *   2) Belirli sure boyunca (max 0.75 sn) MegaTag2 pose tahminleri topla
 *   3) En iyi tahmini (en cok tag, en yakin mesafe) sec
 *   4) Odometry'e yaz -> drivetrain.resetPose(gercekPoz)
 *   5) drivetrain.markVisionSeeded() cagir -> PathPlanner resetPose bypass edilir
 *   6) Tag gorulemezse timeout olur, PathPlanner kendi resetPose'unu kullanir (fallback)
 * ============================================================================
 */
public class VisionAutoSeedCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final String limelightName;
    
    /** Maksimum bekleme suresi (saniye) - auto suresinden yememeli! */
    private static final double MAX_WAIT_SECONDS = 0.75;
    
    /** Minimum tag sayisi kabul icin */
    private static final int MIN_TAG_COUNT = 1;
    
    /** Maksimum tag mesafesi kabul icin (metre) */
    private static final double MAX_TAG_DISTANCE = 5.0;
    
    /** Ambiguity esigi (tek tag icin) */
    private static final double MAX_AMBIGUITY = 0.15;
    private static final double[] TAG_ROTATIONS_DEG = {
        0.0, 180.0, 90.0, 180.0, 180.0, -90.0, 180.0, 0.0, -90.0,
        0.0, 0.0, 90.0, 0.0, 180.0, 180.0, 180.0, 180.0, 0.0, -90.0,
        0.0, 0.0, 90.0, 0.0, 180.0, 90.0, 180.0, 180.0, -90.0, 180.0,
        0.0, 0.0, 0.0, 0.0
    };
    private Timer timer = new Timer();
    private boolean seeded = false;
    
    // En iyi tahmin (birden fazla okuma arasinda en iyisini sec)
    private Pose2d bestPose = null;
    private int bestTagCount = 0;
    private double bestAvgDist = Double.MAX_VALUE;
    private int attemptCount = 0;

    public VisionAutoSeedCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            String limelightName) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.limelightName = limelightName;
        // NOT: drivetrain'i require etmiyoruz cunku sadece resetPose yapiyoruz,
        // surmeye calismiyoruz. Vision da require yok cunku sadece okuma yapiyoruz.
    }

    @Override
    public void initialize() {
        timer.restart();
        seeded = false;
        bestPose = null;
        bestTagCount = 0;
        bestAvgDist = Double.MAX_VALUE;
        attemptCount = 0;
        
        // Vision'i ac
        vision.setEnabled(true);
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
        
        // Seed flag'ini sifirla
        drivetrain.resetVisionSeedFlag();
        
        SmartDashboard.putString("AutoSeed/Status", "SEARCHING...");
    }

    @Override
    public void execute() {
        attemptCount++;
        
        PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // MegaTag2 field heading ister. Seed once MT1 heading guvenilir ise onunla bootstrap et.
        double yawDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        if (hasReliableHeadingEstimate(mt1Estimate)) {
            yawDeg = mt1Estimate.pose.getRotation().getDegrees();
        }
        double yawRateDegPerSec = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(limelightName, yawDeg, yawRateDegPerSec, 0, 0, 0, 0);
        
        // MegaTag2 pose tahmini al
        // KRITIK: MegaTag2 icin HER ZAMAN wpiBlue coordinate system kullan.
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        
        if (estimate == null || estimate.pose == null) return;
        if (estimate.tagCount < MIN_TAG_COUNT) return;
        if (estimate.avgTagDist > MAX_TAG_DISTANCE) return;
        
        // Tek tag ambiguity kontrolu
        if (estimate.tagCount == 1
                && estimate.rawFiducials != null
                && estimate.rawFiducials.length > 0
                && estimate.rawFiducials[0].ambiguity > MAX_AMBIGUITY) {
            return;
        }
        
        // Saha sinirlari icinde mi? (basit kontrol)
        double px = estimate.pose.getX();
        double py = estimate.pose.getY();
        if (px < -0.5 || px > 17.0 || py < -0.5 || py > 8.5) return;
        
        if (!hasReliableHeadingEstimate(mt1Estimate)) return;

        Pose2d xyPose = selectBestVisionPose(estimate, mt1Estimate);

        Pose2d candidatePose = new Pose2d(
            xyPose.getTranslation(),
            getSeedHeading(mt1Estimate, drivetrain.getState().Pose)
        );

        // Bu tahmin daha iyi mi? (daha fazla tag VEYA ayni tag sayisinda daha yakin)
        boolean isBetter = false;
        if (estimate.tagCount > bestTagCount) {
            isBetter = true;
        } else if (estimate.tagCount == bestTagCount && estimate.avgTagDist < bestAvgDist) {
            isBetter = true;
        }
        
        if (isBetter) {
            bestPose = candidatePose;
            bestTagCount = estimate.tagCount;
            bestAvgDist = estimate.avgTagDist;
        }
        
        // 2+ tag gorulduyse veya 0.3 sn gectiyse hemen seed et
        // (tek tag icin biraz daha bekle, belki daha iyi bir okuma gelir)
        if (bestPose != null) {
            if (bestTagCount >= 2 || timer.hasElapsed(0.3)) {
                applySeed();
            }
        }
    }
    
    /**
     * En iyi tahmin edilen pozisyonu odometry'e yazar.
     */
    private void applySeed() {
        if (seeded || bestPose == null) return;
        
        // Odometry'e gercek pozisyonu yaz
        drivetrain.resetPose(bestPose);
        
        // PathPlanner'in resetPose'unu bypass etmek icin flag
        drivetrain.markVisionSeeded();
        
        seeded = true;
        
        DriverStation.reportWarning(
            "[AutoSeed] BASARILI! Gercek pozisyon: " + bestPose.toString() +
            " | Tag sayisi: " + bestTagCount +
            " | Ort. mesafe: " + String.format("%.2fm", bestAvgDist) +
            " | Deneme: " + attemptCount, false);
        
        SmartDashboard.putString("AutoSeed/Status", "SEEDED OK");
        SmartDashboard.putNumber("AutoSeed/X", Math.round(bestPose.getX() * 100.0) / 100.0);
        SmartDashboard.putNumber("AutoSeed/Y", Math.round(bestPose.getY() * 100.0) / 100.0);
        SmartDashboard.putNumber("AutoSeed/Heading", 
            Math.round(bestPose.getRotation().getDegrees() * 10.0) / 10.0);
        SmartDashboard.putNumber("AutoSeed/Tags", bestTagCount);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        
        // Eger seed yapilamadiysa ve bir tahmin varsa, son sansla yap
        if (!seeded && bestPose != null) {
            applySeed();
        }
        
        if (!seeded) {
            DriverStation.reportError(
                "[AutoSeed] BASARISIZ! AprilTag gorulemedi. " +
                "PathPlanner fallback kullanilacak.", false);
            SmartDashboard.putString("AutoSeed/Status", "FAILED - no tags");
        }
        
        // LED'i pipeline kontrolune geri ver
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }

    @Override
    public boolean isFinished() {
        // Seed edildiyse veya zaman dolduysa bitir
        return seeded || timer.hasElapsed(MAX_WAIT_SECONDS);
    }

    private edu.wpi.first.math.geometry.Rotation2d getSeedHeading(PoseEstimate mt1Estimate, Pose2d currentPose) {
        if (hasReliableHeadingEstimate(mt1Estimate)) {
            return mt1Estimate.pose.getRotation();
        }
        return currentPose.getRotation();
    }

    private boolean hasReliableHeadingEstimate(PoseEstimate mt1Estimate) {
        if (mt1Estimate == null || mt1Estimate.pose == null || mt1Estimate.tagCount <= 0) {
            return false;
        }

        return mt1Estimate.tagCount >= 2 && mt1Estimate.avgTagDist < 4.0;
    }

    private Pose2d selectBestVisionPose(PoseEstimate mt2Estimate, PoseEstimate mt1Estimate) {
        if (mt2Estimate == null || mt2Estimate.pose == null) {
            return drivetrain.getState().Pose;
        }

        if (mt2Estimate.tagCount == 1) {
            Pose2d singleTagPose = buildSingleTagPoseFromRobotSpace();
            return singleTagPose != null ? singleTagPose : mt2Estimate.pose;
        }

        RawFiducial[] rawFiducials = mt2Estimate.rawFiducials;
        if (rawFiducials == null || rawFiducials.length < 2 || mt1Estimate == null || mt1Estimate.pose == null) {
            return mt2Estimate.pose;
        }

        double mt2Score = scorePoseCandidate(mt2Estimate.pose, rawFiducials);
        double mt1Score = scorePoseCandidate(mt1Estimate.pose, rawFiducials);
        return mt1Score < mt2Score ? mt1Estimate.pose : mt2Estimate.pose;
    }

    private Pose2d buildSingleTagPoseFromRobotSpace() {
        int tagId = (int) Math.round(LimelightHelpers.getFiducialID(limelightName));
        double[] tagPos = VisionSubsystem.getTagPosition(tagId);
        if (tagPos == null || tagId <= 0 || tagId >= TAG_ROTATIONS_DEG.length) {
            return null;
        }

        Pose2d targetPoseRobotSpace = LimelightHelpers.toPose2D(
            LimelightHelpers.getTargetPose_RobotSpace(limelightName));
        if (targetPoseRobotSpace == null
                || targetPoseRobotSpace.getTranslation().getNorm() < 0.05) {
            return null;
        }

        Pose2d tagFieldPose = new Pose2d(
            tagPos[0],
            tagPos[1],
            Rotation2d.fromDegrees(TAG_ROTATIONS_DEG[tagId]));

        Transform2d robotToTag = new Transform2d(
            targetPoseRobotSpace.getTranslation(),
            targetPoseRobotSpace.getRotation());

        return tagFieldPose.transformBy(robotToTag.inverse());
    }

    private double scorePoseCandidate(Pose2d candidatePose, RawFiducial[] rawFiducials) {
        if (candidatePose == null || rawFiducials == null || rawFiducials.length == 0) {
            return Double.POSITIVE_INFINITY;
        }

        double totalScore = 0.0;
        int used = 0;

        for (RawFiducial fiducial : rawFiducials) {
            if (fiducial == null) continue;
            double[] tagPos = VisionSubsystem.getTagPosition(fiducial.id);
            if (tagPos == null) continue;

            edu.wpi.first.math.geometry.Translation2d tagTranslation =
                new edu.wpi.first.math.geometry.Translation2d(tagPos[0], tagPos[1]);
            edu.wpi.first.math.geometry.Translation2d toTag =
                tagTranslation.minus(candidatePose.getTranslation());

            double expectedDist = toTag.getNorm();
            double distErr = Math.abs(expectedDist - fiducial.distToRobot);

            double relativeBearingDeg = normalizeDeg(
                toTag.getAngle().minus(candidatePose.getRotation()).getDegrees());
            double expectedTxDeg = -relativeBearingDeg;
            double txErr = Math.abs(normalizeDeg(expectedTxDeg - fiducial.txnc));

            totalScore += distErr * 3.0 + txErr * 0.02 + fiducial.ambiguity * 0.5;
            used++;
        }

        return used > 0 ? totalScore / used : Double.POSITIVE_INFINITY;
    }

    private static double normalizeDeg(double degrees) {
        return Math.toDegrees(Math.atan2(
            Math.sin(Math.toRadians(degrees)),
            Math.cos(Math.toRadians(degrees))));
    }
}
