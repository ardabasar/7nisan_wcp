package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * ============================================================================
 * VISION SUBSYSTEM - 2026 REBUILT Saha Telemetrisi + Elastic Dashboard
 * ============================================================================
 * WPILib 2026.2.1 Resmi AprilTag JSON verileri kullanilmaktadir.
 * Kaynak: 2026-rebuilt-welded.json (wpilibsuite/allwpilib v2026.2.1)
 *
 * Saha: REBUILT presented by Haas
 * Saha Boyutu: 16.541m x 8.069m
 * Toplam AprilTag: 32 adet
 * Koordinat Sistemi: Blue alliance duvarinin sag alt kosesi origin (WPILib NWU)
 *
 * ONEMLI: Vision varsayilan olarak ACIK baslar!
 * Robot acilir acilmaz (disabled modda bile) AprilTag tarayarak
 * gercek konumunu tespit eder. x=0 y=0'dan baslamaz.
 * Bu sayede teleop veya otonom basladiginda robot zaten konumunu bilir.
 *
 * Dashboard Verileri (sadece gerekli olanlar):
 *   - Robot Pozisyonu: X, Y, Heading
 *   - Hub Mesafesi: Alliance Hub'ina olan mesafe
 *   - Gorulen Tag: ID, isim, mesafe
 *   - Saha Bolgesi: Robotun bulundugu alan
 *
 * Elastic Dashboard Kurulumu:
 *   1) Elastic'te "+" ile yeni widget ekle
 *   2) "RobotPosition" tablosundan alanlari sec
 *   3) Field2d icin: SmartDashboard -> "Field" widget'ini surukle
 * ============================================================================
 */
public class VisionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;

    /** Vision varsayilan olarak ACIK baslar - robot nerede oldugunu HEMEN bilsin */
    private boolean enabled = true;

    // ========================================================================
    // GUVENLIK FILTRELERI
    // ========================================================================
    private static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(720);
    private static final double MAX_AVG_TAG_DISTANCE_METERS      = 4.0;
    private static final double MAX_SINGLE_TAG_AMBIGUITY         = 0.30;

    // StdDev matrisleri - onceden hesaplanmis
    private static final Matrix<N3, N1> STD_DEV_1_TAG =
        VecBuilder.fill(0.7,  0.7,  Math.toRadians(12));
    private static final Matrix<N3, N1> STD_DEV_2_TAG =
        VecBuilder.fill(0.45, 0.45, Math.toRadians(8));
    private static final Matrix<N3, N1> STD_DEV_3_TAG =
        VecBuilder.fill(0.30, 0.30, Math.toRadians(6));

    // Vision seed tracking - odometry en az 1 kez vision ile guncellendi mi?
    private boolean visionSeeded = false;
    private int visionUpdateCount = 0;

    // Pose jump rejection - ani sicrama korumasi
    private Pose2d lastAcceptedPose = null;
    private double lastAcceptedTimestampSeconds = -1.0;
    private static final double MAX_POSE_JUMP_METERS = 3.5;
    private static final double MAX_VISION_GAP_FOR_JUMP_CHECK_SECONDS = 1.0;
    private static final double MAX_REACQUIRE_ODOM_DIFF_METERS = 6.0;

    // SmartDashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // Alliance cache
    private boolean isRedAlliance = false;
    private int allianceCacheCounter = ALLIANCE_CHECK_INTERVAL;
    private static final int ALLIANCE_CHECK_INTERVAL = 250;

    // ========================================================================
    // LIMELIGHT KAMERA STREAM - Elastic Dashboard'da canli goruntu
    // ========================================================================
    // Elastic'te "CameraServer" tablosundan otomatik gorunur.
    // Veya manuel: + -> CameraStream widget ekle
    // Limelight varsayilan MJPEG stream: http://limelight.local:5800/stream.mjpg
    // ========================================================================

    // ========================================================================
    // FIELD2D - Elastic Dashboard icin saha gorunumu
    // ========================================================================
    private final Field2d field2d = new Field2d();

    // ========================================================================
    // NETWORKTABLES PUBLISHERS
    // ========================================================================
    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

    // --- ANA TABLO: RobotPosition (Elastic'te her zaman gorunecek) ---
    private final NetworkTable posTable = ntInst.getTable("RobotPosition");
    private final DoublePublisher pubDistRedHub  = posTable.getDoubleTopic("Red Hub Mesafe (m)").publish();
    private final DoublePublisher pubDistBlueHub = posTable.getDoubleTopic("Blue Hub Mesafe (m)").publish();
    private final StringPublisher pubZone = posTable.getStringTopic("Saha Bolgesi").publish();
    private final DoublePublisher pubTagId     = posTable.getDoubleTopic("Gorulen Tag ID").publish();
    private final DoublePublisher pubTagDist   = posTable.getDoubleTopic("Tag Mesafe (m)").publish();
    private final DoublePublisher pubX       = posTable.getDoubleTopic("X (m)").publish();
    private final DoublePublisher pubY       = posTable.getDoubleTopic("Y (m)").publish();
    private final DoublePublisher pubHeading = posTable.getDoubleTopic("Heading (deg)").publish();

    // --- DEBUG TABLO: Diagnostics (ayri tablo - test/debug icin) ---
    private final NetworkTable debugTable = ntInst.getTable("Debug");
    private final DoublePublisher dbgHubDist      = debugTable.getDoubleTopic("Hub Mesafe (m)").publish();
    private final DoublePublisher dbgHubDistInch   = debugTable.getDoubleTopic("Hub Mesafe (inch)").publish();
    private final DoublePublisher dbgLLDirectDist  = debugTable.getDoubleTopic("LL Direkt Mesafe (m)").publish();
    private final DoublePublisher dbgLLTx          = debugTable.getDoubleTopic("LL TX").publish();
    private final DoublePublisher dbgLLTy          = debugTable.getDoubleTopic("LL TY").publish();
    private final DoublePublisher dbgLLTa          = debugTable.getDoubleTopic("LL TA").publish();

    // ========================================================================
    // 2026 REBUILT SAHA KOORDINATLARI (metre cinsinden)
    // Kaynak: WPILib 2026.2.1 - 2026-rebuilt-welded.json
    // Koordinat Sistemi: Blue alliance duvarinin sag alt kosesi (0,0)
    //   +X -> Red alliance yonune dogru
    //   +Y -> Scoring table (seyirci) yonune dogru
    // Saha boyutu: 16.541m x 8.069m
    // ========================================================================

    // --- Saha boyutlari ---
    private static final double FIELD_LENGTH = 16.541; // metre
    private static final double FIELD_WIDTH  = 8.069;  // metre

    // --- AprilTag Pozisyonlari (ID -> X,Y metre) ---
    // WPILib 2026.2.1 resmi verileri (2026-rebuilt-welded.json)
    private static final double[][] TAG_POSITIONS = {
        // index 0 bos (ID'ler 1'den baslar)
        {0, 0},
        // === RED ALLIANCE TARAFI ===
        // ID  1: Trench, Red
        {11.878, 7.425},
        // ID  2: Hub, Red (sag yuz - +90 derece)
        {11.915, 4.638},
        // ID  3: Hub, Red (on yuz - 180 derece)
        {11.312, 4.390},
        // ID  4: Hub, Red (on yuz - 180 derece)
        {11.312, 4.035},
        // ID  5: Hub, Red (sol yuz - -90 derece)
        {11.915, 3.431},
        // ID  6: Outpost, Red (on yuz - 180 derece)
        {11.878, 0.645},
        // ID  7: Outpost, Red (arka yuz - 0 derece)
        {11.953, 0.645},
        // ID  8: Hub, Red (sol yuz - -90 derece, dis)
        {12.271, 3.431},
        // ID  9: Hub, Red (arka yuz - 0 derece, dis)
        {12.519, 3.679},
        // ID 10: Hub, Red (arka yuz - 0 derece, dis)
        {12.519, 4.035},
        // ID 11: Hub, Red (sag yuz - +90 derece, dis)
        {12.271, 4.638},
        // ID 12: Trench, Red (arka yuz - 0 derece)
        {11.953, 7.425},

        // === RED ALLIANCE WALL (TOWER) ===
        // ID 13: Tower, Red (on yuz - 180 derece)
        {16.533, 7.403},
        // ID 14: Tower, Red (on yuz - 180 derece)
        {16.533, 6.972},
        // ID 15: Tower, Red (on yuz - 180 derece)
        {16.533, 4.324},
        // ID 16: Tower, Red (on yuz - 180 derece)
        {16.533, 3.892},

        // === BLUE ALLIANCE TARAFI ===
        // ID 17: Outpost, Blue (arka yuz - 0 derece)
        {4.663, 0.645},
        // ID 18: Hub, Blue (sol yuz - -90 derece)
        {4.626, 3.431},
        // ID 19: Hub, Blue (arka yuz - 0 derece, dis)
        {5.229, 3.679},
        // ID 20: Hub, Blue (arka yuz - 0 derece, dis)
        {5.229, 4.035},
        // ID 21: Hub, Blue (sag yuz - +90 derece, dis)
        {4.626, 4.638},
        // ID 22: Trench, Blue (arka yuz - 0 derece)
        {4.663, 7.425},
        // ID 23: Trench, Blue (on yuz - 180 derece)
        {4.588, 7.425},
        // ID 24: Hub, Blue (sag yuz - +90 derece)
        {4.270, 4.638},
        // ID 25: Hub, Blue (on yuz - 180 derece)
        {4.022, 4.390},
        // ID 26: Hub, Blue (on yuz - 180 derece)
        {4.022, 4.035},
        // ID 27: Hub, Blue (sol yuz - -90 derece)
        {4.270, 3.431},
        // ID 28: Outpost, Blue (on yuz - 180 derece)
        {4.588, 0.645},

        // === BLUE ALLIANCE WALL (TOWER) ===
        // ID 29: Tower, Blue (arka yuz - 0 derece)
        {0.008, 0.666},
        // ID 30: Tower, Blue (arka yuz - 0 derece)
        {0.008, 1.098},
        // ID 31: Tower, Blue (arka yuz - 0 derece)
        {0.008, 3.746},
        // ID 32: Tower, Blue (arka yuz - 0 derece)
        {0.008, 4.178},
    };

    // --- Tag ID -> Isim Haritasi ---
    private static final String[] TAG_NAMES = {
        "",                                 //  0 - yok
        "Red Trench (On)",                  //  1
        "Red Hub (Sag Yuz)",                //  2
        "Red Hub (On-Ust)",                 //  3
        "Red Hub (On-Alt)",                 //  4
        "Red Hub (Sol Yuz)",                //  5
        "Red Outpost (On)",                 //  6
        "Red Outpost (Arka)",               //  7
        "Red Hub (Sol Dis)",                //  8
        "Red Hub (Arka-Alt)",               //  9
        "Red Hub (Arka-Ust)",               // 10
        "Red Hub (Sag Dis)",                // 11
        "Red Trench (Arka)",                // 12
        "Red Tower (Ust-1)",                // 13
        "Red Tower (Ust-2)",                // 14
        "Red Tower (Alt-1)",                // 15
        "Red Tower (Alt-2)",                // 16
        "Blue Outpost (Arka)",              // 17
        "Blue Hub (Sol Yuz)",               // 18
        "Blue Hub (Arka-Alt)",              // 19
        "Blue Hub (Arka-Ust)",              // 20
        "Blue Hub (Sag Dis)",               // 21
        "Blue Trench (Arka)",               // 22
        "Blue Trench (On)",                 // 23
        "Blue Hub (Sag Yuz)",               // 24
        "Blue Hub (On-Ust)",                // 25
        "Blue Hub (On-Alt)",                // 26
        "Blue Hub (Sol Dis)",               // 27
        "Blue Outpost (On)",                // 28
        "Blue Tower (Alt-1)",               // 29
        "Blue Tower (Alt-2)",               // 30
        "Blue Tower (Ust-1)",               // 31
        "Blue Tower (Ust-2)",               // 32
    };

    // ========================================================================
    // HUB MERKEZLERI - FUEL skorlama hedefi
    // 2026 REBUILT sahasi resmi cizim + WCP referansina gore sabit nokta.
    // Bu degerler WCP Landmarks.java ile birebir uyumludur.
    // ========================================================================

    // WCP/field drawing:
    //   Blue Hub: (182.105 in, 158.845 in) -> (4.625467 m, 4.034663 m)
    //   Red  Hub: (469.115 in, 158.845 in) -> (11.915521 m, 4.034663 m)
    // Driver station duvarina dik mesafe her iki alliance icin ~182.1 in (4.625 m)
    private static final Translation2d RED_HUB_CENTER = new Translation2d(
        11.915521,
        4.034663
    );
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(
        4.625467,
        4.034663
    );

    // ========================================================================
    // KAMERA POZISYONU - Robot merkezine gore (metre + derece)
    // ========================================================================
    // Robotun swerve modulu merkeze 0.30m (11.811").
    // Kamera robotun onunden 40cm geride → merkezden ~10cm arkada.
    //
    //   Forward (X):  -0.10  → merkezden 10cm arkada
    //   Side    (Y):   0.00  → ortada (sag/sol kayma yok)
    //   Up      (Z):   0.66  → YERDEN 66cm (sase 10cm + saseden 56cm)
    //   Roll:           0.0  → kamera duz (yana yatik degil)
    //   Pitch:         15.0  → 15 derece yukari bakiyor
    //   Yaw:            0.0  → robotun on yonune bakiyor
    //
    // ONEMLI: Z degeri YERDEN yukseklik olmali, saseden degil!
    //   Tekerlek yaricapi: 3.54" (~9cm) + sase kalinligi ~1cm = ~10cm
    //   Kamera saseden 56cm → yerden 56 + 10 = 66cm
    // ========================================================================
    private static final double CAM_FORWARD_M = -0.10;   // merkezden arkaya (-)
    private static final double CAM_SIDE_M    =  0.00;   // merkezde
    private static final double CAM_UP_M      =  0.66;   // YERDEN yukseklik (sase+56cm)
    private static final double CAM_ROLL_DEG  =  0.0;
    private static final double CAM_PITCH_DEG = 15.0;    // yukari bakis acisi
    private static final double CAM_YAW_DEG   =  0.0;    // on yone bakiyor

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain    = drivetrain;
        this.limelightName = limelightName;

        // =============================================================
        // KRITIK: Kamera pozisyonunu Limelight'a bildir
        // MegaTag2 bu bilgiyi kullanarak robotun saha pozisyonunu hesaplar.
        // Bu ayarlanmazsa odometry HEP YANLIS olur!
        // =============================================================
        LimelightHelpers.setCameraPose_RobotSpace(limelightName,
            CAM_FORWARD_M,   // Forward (X): merkezden ileri/geri
            CAM_SIDE_M,      // Side (Y): merkezden sag/sol
            CAM_UP_M,        // Up (Z): yerden yukseklik
            CAM_ROLL_DEG,    // Roll: yana yatma
            CAM_PITCH_DEG,   // Pitch: yukari/asagi bakma
            CAM_YAW_DEG      // Yaw: saga/sola bakma
        );

        System.out.println("[Vision] Kamera pozisyonu ayarlandi: "
            + "X=" + CAM_FORWARD_M + "m, Y=" + CAM_SIDE_M + "m, Z=" + CAM_UP_M + "m, "
            + "Pitch=" + CAM_PITCH_DEG + "°");

        // Field2d widget'ini SmartDashboard'a kaydet
        SmartDashboard.putData("Field", field2d);

        // Limelight kamera stream'ini Elastic'e kaydet
        String streamUrl = "http://" + limelightName + ".local:5800/stream.mjpg";
        var cameraTable = ntInst.getTable("CameraPublisher").getSubTable(limelightName);
        cameraTable.getEntry("streams").setStringArray(new String[]{
            "mjpg:" + streamUrl
        });
        SmartDashboard.putString("limelight/StreamURL", streamUrl);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        SmartDashboard.putBoolean("Vision/Enabled", enabled);
    }

    public boolean isEnabled() {
        return enabled;
    }

    // ========================================================================
    // PERIODIC - Her loop'ta calisir
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;

        // ==================================================================
        // ALLIANCE CACHE - Her zaman guncellenir (vision kapali olsa bile!)
        // Simulasyonda ve robotta alliance bilgisi FMS/DS'den gelir.
        // ==================================================================
        allianceCacheCounter++;
        if (allianceCacheCounter >= ALLIANCE_CHECK_INTERVAL) {
            allianceCacheCounter = 0;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                isRedAlliance = (alliance.get() == DriverStation.Alliance.Red);
            }
        }

        // ==================================================================
        // KONUM TELEMETRISI - HERZAMAN CALISIR (vision kapali olsa bile!)
        // Odometry her zaman aktif, sadece vision fuzyonu kapanir.
        // ==================================================================
        Pose2d currentPose = drivetrain.getState().Pose;

        // Field2d her loop'ta guncellenir (Elastic'te smooth gozukur)
        field2d.setRobotPose(currentPose);

        // Throttled telemetri (5Hz)
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            double robotX = currentPose.getX();
            double robotY = currentPose.getY();
            double robotHeading = currentPose.getRotation().getDegrees();

            // 1) Hub mesafeleri (en ustte)
            Translation2d robotPos = currentPose.getTranslation();
            double dRedHub  = robotPos.getDistance(RED_HUB_CENTER);
            double dBlueHub = robotPos.getDistance(BLUE_HUB_CENTER);
            pubDistRedHub.set(round2(dRedHub));
            pubDistBlueHub.set(round2(dBlueHub));

            // 2) Atis mesafesi (en yakin hub) - HER ZAMAN gorunur
            double ownHubDist = getDistanceToOwnHub();
            dbgHubDist.set(round2(ownHubDist));
            dbgHubDistInch.set(Math.round(ownHubDist * 39.3701));
            SmartDashboard.putBoolean("Vision/Seeded", visionSeeded);

            // 3) Limelight canli durum - Debug tablosuna yazilir
            boolean hasTag = LimelightHelpers.getTV(limelightName);
            SmartDashboard.putBoolean("Limelight/HasTarget", hasTag);
            if (hasTag) {
                double tx = LimelightHelpers.getTX(limelightName);
                double ty = LimelightHelpers.getTY(limelightName);
                double ta = LimelightHelpers.getTA(limelightName);
                dbgLLTx.set(round2(tx));
                dbgLLTy.set(round2(ty));
                dbgLLTa.set(round2(ta));
                // Direkt kamera mesafesi (tag gorunuyorsa)
                double[] camPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
                if (camPose != null && camPose.length >= 3 && camPose[2] > 0.05) {
                    double directDist = Math.sqrt(camPose[0] * camPose[0] + camPose[2] * camPose[2]);
                    dbgLLDirectDist.set(round2(directDist));
                }
            }

            // 4) Saha bolgesi
            String zone = getFieldZone(robotX, robotY);
            pubZone.set(zone);

            // 5) Gorulen tag bilgisi
            updateTagTelemetry();

            // 6) Robot konum (en altta)
            pubX.set(round2(robotX));
            pubY.set(round2(robotY));
            pubHeading.set(Math.round(robotHeading * 10.0) / 10.0);
        }

        // ==================================================================
        // VISION FUZYONU - Sadece enable ise calisir
        // ==================================================================
        if (!enabled) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "DISABLED");
            }
            return;
        }

        // Robot yaw -> Limelight (MegaTag2 icin)
        double yawDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation(limelightName, yawDeg, 0, 0, 0, 0, 0);

        // Limelight ham veri kontrolu - tag goruyor mu?
        boolean llHasTarget = LimelightHelpers.getTV(limelightName);
        double llTagId = LimelightHelpers.getFiducialID(limelightName);

        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putBoolean("Vision/LL_HasTarget", llHasTarget);
            SmartDashboard.putNumber("Vision/LL_TagID", llTagId);
            SmartDashboard.putNumber("Vision/LL_Yaw", round2(yawDeg));
            SmartDashboard.putString("Vision/Alliance", isRedAlliance ? "RED" : "BLUE");
        }

        // Donen robot kontrolu
        double omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
        if (Math.abs(omega) > MAX_ANGULAR_VELOCITY_RAD_PER_SEC) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "Skip:Spinning");
            }
            return;
        }

        // Pose tahmini al
        PoseEstimate estimate = isRedAlliance
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // ============ DETAYLI REJECTION TESHISI ============
        if (estimate == null || estimate.pose == null) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "REJECT:NullEstimate");
            }
            return;
        }
        if (estimate.tagCount <= 0) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "REJECT:NoTags(tagCount=0)");
            }
            return;
        }
        if (estimate.timestampSeconds <= 0) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "REJECT:BadTimestamp");
            }
            return;
        }
        if (estimate.avgTagDist > MAX_AVG_TAG_DISTANCE_METERS) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status",
                    "REJECT:TooFar(" + round2(estimate.avgTagDist) + "m>4m)");
            }
            return;
        }

        if (estimate.tagCount == 1
                && estimate.rawFiducials != null
                && estimate.rawFiducials.length > 0
                && estimate.rawFiducials[0].ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status",
                    "REJECT:Ambiguity(" + round2(estimate.rawFiducials[0].ambiguity) + ">0.2)");
            }
            return;
        }

        // ==================================================================
        // SAHA SINIR KONTROLU - Pose saha disindaysa reddet
        // ±0.5m tolerans ile (robot kismen duvara yakin olabilir)
        // ==================================================================
        if (estimate.pose.getX() < -0.5 || estimate.pose.getX() > FIELD_LENGTH + 0.5
                || estimate.pose.getY() < -0.5 || estimate.pose.getY() > FIELD_WIDTH + 0.5) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "Skip:OutOfField");
            }
            return;
        }

        // ==================================================================
        // ANI SICRAMA KORUMASI (KILITLENME DAYANIKLI)
        // - Kisa aralikli olcumlerde: son kabul edilen vision pose'una gore jump filtrele
        // - Uzun tag kaybindan sonra: jump filtresini gecici devre disi birak,
        //   odometry'den asiri uzak outlier'lari ele
        // ==================================================================
        final double visionGapSeconds = lastAcceptedTimestampSeconds > 0
            ? estimate.timestampSeconds - lastAcceptedTimestampSeconds
            : Double.POSITIVE_INFINITY;

        if (lastAcceptedPose != null && visionGapSeconds <= MAX_VISION_GAP_FOR_JUMP_CHECK_SECONDS) {
            double jump = estimate.pose.getTranslation()
                .getDistance(lastAcceptedPose.getTranslation());
            if (jump > MAX_POSE_JUMP_METERS) {
                if (loopCount % DASHBOARD_INTERVAL == 0) {
                    SmartDashboard.putString("Vision/Status", "Skip:PoseJump");
                }
                return;
            }
        }

        if (visionGapSeconds > MAX_VISION_GAP_FOR_JUMP_CHECK_SECONDS) {
            double odomDiff = estimate.pose.getTranslation()
                .getDistance(currentPose.getTranslation());
            if (odomDiff > MAX_REACQUIRE_ODOM_DIFF_METERS) {
                if (loopCount % DASHBOARD_INTERVAL == 0) {
                    SmartDashboard.putString("Vision/Status", "Skip:ReacquireOutlier");
                }
                return;
            }
        }

        lastAcceptedPose = estimate.pose;
        lastAcceptedTimestampSeconds = estimate.timestampSeconds;

        // ==================================================================
        // MESAFE BAZLI STDDEV OLCEKLEME
        // Tag count + mesafe carpani ile guven ayarlama
        // ==================================================================
        Matrix<N3, N1> baseStdDevs;
        if      (estimate.tagCount >= 3) baseStdDevs = STD_DEV_3_TAG;
        else if (estimate.tagCount >= 2) baseStdDevs = STD_DEV_2_TAG;
        else                             baseStdDevs = STD_DEV_1_TAG;

        // Uzak tag = daha az guvenilir
        double distFactor = 1.0;
        if (estimate.avgTagDist > 3.0)      distFactor = 2.5;
        else if (estimate.avgTagDist > 2.0) distFactor = 1.5;

        Matrix<N3, N1> scaledStdDevs = VecBuilder.fill(
            baseStdDevs.get(0, 0) * distFactor,
            baseStdDevs.get(1, 0) * distFactor,
            baseStdDevs.get(2, 0)  // Rotation std dev mesafeden etkilenmesin
        );

        // ==================================================================
        // VISION GAP COMPENSATION
        // Tag goremedigimiz sure boyunca odometry drift eder.
        // Uzun araliktan sonra vision'a daha cok guven (StdDevs dusur).
        // Bu sayede robot tag gormediginde kaybolan pozisyonunu
        // tekrar tag gordugunde HIZLICA duzeltir.
        // ==================================================================
        if (!visionSeeded) {
            // ILK SEED: Vision'a maksimum guven - pozisyonu olustur
            scaledStdDevs = VecBuilder.fill(0.05, 0.05, Math.toRadians(2));
        } else if (lastAcceptedTimestampSeconds > 0) {
            double timeSinceLastVision = estimate.timestampSeconds - lastAcceptedTimestampSeconds;
            if (timeSinceLastVision > 2.0) {
                // 2+ saniye tag gorememisiz - odometry cok kaymis olabilir
                // Vision'a cok guven (snap to vision)
                scaledStdDevs = VecBuilder.fill(0.10, 0.10, Math.toRadians(3));
            } else if (timeSinceLastVision > 1.0) {
                // 1-2 saniye gap - orta seviye guven
                scaledStdDevs = VecBuilder.fill(
                    scaledStdDevs.get(0, 0) * 0.3,
                    scaledStdDevs.get(1, 0) * 0.3,
                    scaledStdDevs.get(2, 0) * 0.5
                );
            }
        }

        // Odometry'e ekle
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, scaledStdDevs);
        visionSeeded = true;
        visionUpdateCount++;

        // Vision status
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Vision/Status",  "OK-MT2");
            SmartDashboard.putNumber("Vision/Tags",    estimate.tagCount);
            SmartDashboard.putNumber("Vision/AvgDist", estimate.avgTagDist);
            SmartDashboard.putNumber("Vision/PoseX",   estimate.pose.getX());
            SmartDashboard.putNumber("Vision/PoseY",   estimate.pose.getY());
            SmartDashboard.putNumber("Vision/UpdateCount", visionUpdateCount);
        }
    }

    // ========================================================================
    // TAG TELEMETRI - Limelight'in gordugu tag bilgisi
    // ========================================================================
    private void updateTagTelemetry() {
        // Once getTV ile tag gorunuyor mu kontrol et
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        if (!hasTarget) {
            pubTagId.set(-1);
            pubTagDist.set(-1);
            return;
        }

        // getFiducialID: Limelight'in gordugu ana tag ID'si
        double tagId = LimelightHelpers.getFiducialID(limelightName);

        // Bazi Limelight versiyonlari 0 veya -1 dondurur
        // getRawFiducials ile daha guvenilir veri al
        if (tagId < 1 || tagId > 32) {
            try {
                var rawFids = LimelightHelpers.getRawFiducials(limelightName);
                if (rawFids != null && rawFids.length > 0) {
                    tagId = rawFids[0].id;
                }
            } catch (Exception e) {
                // Hata → sessizce devam et
            }
        }

        if (tagId < 1 || tagId > 32) {
            pubTagId.set(-1);
            pubTagDist.set(-1);
            return;
        }

        int id = (int) tagId;
        pubTagId.set(id);

        // Kamera mesafesi
        double[] camPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
        if (camPose != null && camPose.length >= 3 && camPose[2] > 0.05) {
            double dist = Math.sqrt(camPose[0] * camPose[0] + camPose[2] * camPose[2]);
            pubTagDist.set(round2(dist));
        } else {
            // Fallback: ta'dan mesafe tahmini
            double ta = LimelightHelpers.getTA(limelightName);
            if (ta > 0.01) {
                pubTagDist.set(round2(25.0 / Math.sqrt(ta)));
            } else {
                pubTagDist.set(-1);
            }
        }
    }

    // ========================================================================
    // SAHA BOLGE TESPITI - 2026 REBUILT
    // ========================================================================
    private String getFieldZone(double x, double y) {
        // Saha: 16.541m x 8.069m

        // --- BLUE ALLIANCE AREA (x < ~2.5m) ---
        if (x < 2.5) {
            // Blue Tower bolgesi (tower alliance wall'da, x ~0)
            if (y < 2.0) return "Blue Tower Bolgesi (Alt)";
            if (y > 2.0 && y < 5.0) return "Blue Tower Bolgesi (Orta)";
            return "Blue Alliance Area";
        }

        // --- BLUE ELEMENT BOLGESI (x ~2.5 - 6.5) ---
        if (x >= 2.5 && x < 6.5) {
            // Blue Hub bolgesi (merkez ~4.3, 4.0)
            double hubDist = Math.sqrt(
                Math.pow(x - BLUE_HUB_CENTER.getX(), 2) +
                Math.pow(y - BLUE_HUB_CENTER.getY(), 2)
            );
            if (hubDist < 2.0) return "Blue Hub Bolgesi";

            // Blue Outpost (y ~0.645, x ~4.6)
            if (y < 1.5 && x > 3.5 && x < 5.5) return "Blue Outpost Bolgesi";

            // Blue Trench (y ~7.425, x ~4.6)
            if (y > 6.5 && x > 3.5 && x < 5.5) return "Blue Trench Bolgesi";

            return "Blue Alliance Bolgesi";
        }

        // --- RED ELEMENT BOLGESI (x ~10.0 - 14.0) ---
        if (x >= 10.0 && x < 14.0) {
            // Red Hub bolgesi (merkez ~11.6, 4.1)
            double hubDist = Math.sqrt(
                Math.pow(x - RED_HUB_CENTER.getX(), 2) +
                Math.pow(y - RED_HUB_CENTER.getY(), 2)
            );
            if (hubDist < 2.0) return "Red Hub Bolgesi";

            // Red Outpost (y ~0.645, x ~11.9)
            if (y < 1.5 && x > 10.5 && x < 13.0) return "Red Outpost Bolgesi";

            // Red Trench (y ~7.425, x ~11.9)
            if (y > 6.5 && x > 10.5 && x < 13.0) return "Red Trench Bolgesi";

            return "Red Alliance Bolgesi";
        }

        // --- RED ALLIANCE AREA (x > ~14.0) ---
        if (x >= 14.0) {
            // Red Tower bolgesi (tower alliance wall'da, x ~16.5)
            if (x > 15.0 && y < 5.0 && y > 2.5) return "Red Tower Bolgesi (Alt)";
            if (x > 15.0 && y >= 5.0) return "Red Tower Bolgesi (Ust)";
            return "Red Alliance Area";
        }

        // --- NEUTRAL ZONE / ORTA SAHA (x ~6.5 - 10.0) ---
        if (y < 1.5) return "Neutral Zone (Alt)";
        if (y > 6.5) return "Neutral Zone (Ust)";
        return "Neutral Zone (Orta)";
    }

    // ========================================================================
    // YARDIMCI METODLAR
    // ========================================================================
    private static double round2(double val) {
        return Math.round(val * 100.0) / 100.0;
    }

    /**
     * Belirli bir AprilTag'e olan mesafeyi dondurur.
     * Drivetrain pose'u kullanarak hesaplar (kamera gorunmese bile).
     *
     * @param tagId AprilTag ID (1-32)
     * @return Mesafe metre cinsinden, gecersiz ID icin -1
     */
    public double getDistanceToTag(int tagId) {
        if (tagId < 1 || tagId >= TAG_POSITIONS.length) return -1;
        Pose2d pose = drivetrain.getState().Pose;
        double dx = pose.getX() - TAG_POSITIONS[tagId][0];
        double dy = pose.getY() - TAG_POSITIONS[tagId][1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Red Hub'a olan mesafeyi dondurur.
     */
    public double getDistanceToRedHub() {
        Pose2d pose = drivetrain.getState().Pose;
        return pose.getTranslation().getDistance(RED_HUB_CENTER);
    }

    /**
     * Blue Hub'a olan mesafeyi dondurur.
     */
    public double getDistanceToBlueHub() {
        Pose2d pose = drivetrain.getState().Pose;
        return pose.getTranslation().getDistance(BLUE_HUB_CENTER);
    }

    /**
     * Atis yapilacak Hub'a olan mesafeyi dondurur.
     *
     * Oncelik sirasi:
     *   1) FMS/Driver Station'dan alliance bilindigi zaman -> kendi hub'ina mesafe
     *   2) Alliance bilinmiyorsa (pratik ortam) -> EN YAKIN hub'a mesafe
     *
     * Bu sayede:
     *   - Musabakada: FMS otomatik alliance atar, dogru hub secilir
     *   - Pratikte: Alliance secmeseniz bile en yakin hub'a gore hesaplar
     */
    public double getDistanceToOwnHub() {
        Pose2d pose = drivetrain.getState().Pose;
        Translation2d robotPos = pose.getTranslation();

        // Alliance biliniyorsa (FMS bagli veya DS'de manuel secilmis)
        if (DriverStation.getAlliance().isPresent()) {
            Translation2d hub = isRedAlliance ? RED_HUB_CENTER : BLUE_HUB_CENTER;
            return robotPos.getDistance(hub);
        }

        // Alliance bilinmiyorsa (pratik ortam) -> en yakin hub'a mesafe
        double distRed  = robotPos.getDistance(RED_HUB_CENTER);
        double distBlue = robotPos.getDistance(BLUE_HUB_CENTER);
        return Math.min(distRed, distBlue);
    }

    /**
     * Vision ile odometry en az 1 kez guncellendi mi?
     * ShootCommand bunu kontrol eder - false ise odometry mesafesine guvenme.
     */
    public boolean hasBeenSeeded() {
        return visionSeeded;
    }

    /**
     * Alliance bilgisini dondurur.
     */
    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /** Red Hub merkez koordinatini dondurur (AlignToAprilTag icin). */
    public static Translation2d getRedHubCenter() {
        return RED_HUB_CENTER;
    }

    /** Blue Hub merkez koordinatini dondurur (AlignToAprilTag icin). */
    public static Translation2d getBlueHubCenter() {
        return BLUE_HUB_CENTER;
    }

    /**
     * Tag pozisyonlarini dondurur (diger komutlarda kullanilabilir).
     */
    public static double[] getTagPosition(int tagId) {
        if (tagId < 1 || tagId >= TAG_POSITIONS.length) return null;
        return TAG_POSITIONS[tagId];
    }

    /**
     * Tag ismini dondurur.
     */
    public static String getTagName(int tagId) {
        if (tagId < 0 || tagId >= TAG_NAMES.length) return "Bilinmeyen";
        return TAG_NAMES[tagId];
    }
}
