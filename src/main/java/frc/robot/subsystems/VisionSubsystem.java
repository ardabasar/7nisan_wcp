package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.LimelightHelpers.RawFiducial;

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
    private static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.toRadians(360);  // sadece cok hizli donuste skip
    private static final double MAX_AVG_TAG_DISTANCE_METERS      = 5.0;                  // 5m'ye kadar tag kabul et
    private static final double MAX_SINGLE_TAG_AMBIGUITY         = 0.3;                  // tek tag ambiguity filtresi
    private static final double BUMP_PITCH_THRESHOLD_DEG         = 5.0;                  // pitch > 5° → robot tumsekte

    // ========================================================================
    // STDDEV HESABI (Turkiye 1.si + Dunya Finalisti hibrit)
    // ========================================================================
    // MegaTag2 → XY icin: xyStdDev = 0.125 * avgTagDist^2 / tagCount, heading = 999999
    // MegaTag1 → SADECE ilk seed ve sert toparlamada heading adayidir.
    //            Surekli heading fusion acik tutulmaz; aciyi gyro tasir.
    // Referans: 2025 Turkiye sampiyonu + dunya finalisti hibrit
    private static final double XY_STDDEV_COEFFICIENT = 0.125;
    private static final double HEADING_STDDEV_MULTI_TAG = 0.1;  // 2+ tag → heading duzelt (~6 derece)
    private static final double HEADING_STDDEV_SINGLE_TAG = 999999;  // 1 tag → heading'e dokunma

    // Vision seed tracking
    private boolean visionSeeded = false;
    private int visionUpdateCount = 0;

    // Son kabul edilen vision verisi (jump rejection icin)
    private Pose2d lastAcceptedPose = null;
    private double lastAcceptedTimestampSeconds = -1.0;

    // Hizalama aktifken resetPose YAPMA (titreme onleme)
    // AlignToHubOdometry gibi komutlar bunu true yapar
    private boolean alignmentActive = false;

    // SmartDashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // Alliance cache
    private boolean isRedAlliance = false;
    private int allianceCacheCounter = ALLIANCE_CHECK_INTERVAL;
    private static final int ALLIANCE_CHECK_INTERVAL = 10;  // Hizli guncelle (250 cok yavas!)

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

    // --- LIMELIGHT DURUM TABLO: Elastic Dashboard'da kolay gorulebilir ---
    private final NetworkTable llTable = ntInst.getTable("Limelight");
    private final StringPublisher llStatus    = llTable.getStringTopic("Durum").publish();
    private final DoublePublisher llTagCount  = llTable.getDoubleTopic("Tag Sayisi").publish();
    private final DoublePublisher llTagDist   = llTable.getDoubleTopic("Tag Mesafe (m)").publish();
    private final DoublePublisher llFPS       = llTable.getDoubleTopic("FPS").publish();
    private final DoublePublisher llPipeline  = llTable.getDoubleTopic("Pipeline").publish();
    private final DoublePublisher llLatency   = llTable.getDoubleTopic("Gecikme (ms)").publish();

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
    // Resmi saha verileri (2026-rebuilt-andymark.json)
    private static final double[][] TAG_POSITIONS = {
        // index 0 bos (ID'ler 1'den baslar)
        {0, 0},
        // === RED ALLIANCE TARAFI ===
        // ID  1: Trench, Red
        {11.863959, 7.4114914},
        // ID  2: Hub, Red (sag yuz - +90 derece)
        {11.9013986, 4.6247558},
        // ID  3: Hub, Red (on yuz - 180 derece)
        {11.2978438, 4.3769534},
        // ID  4: Hub, Red (on yuz - 180 derece)
        {11.2978438, 4.0213534},
        // ID  5: Hub, Red (sol yuz - -90 derece)
        {11.9013986, 3.417951},
        // ID  6: Outpost, Red (on yuz - 180 derece)
        {11.863959, 0.6312154},
        // ID  7: Outpost, Red (arka yuz - 0 derece)
        {11.9388636, 0.6312154},
        // ID  8: Hub, Red (sol yuz - -90 derece, dis)
        {12.2569986, 3.417951},
        // ID  9: Hub, Red (arka yuz - 0 derece, dis)
        {12.5051566, 3.6657534},
        // ID 10: Hub, Red (arka yuz - 0 derece, dis)
        {12.5051566, 4.0213534},
        // ID 11: Hub, Red (sag yuz - +90 derece, dis)
        {12.2569986, 4.6247558},
        // ID 12: Trench, Red (arka yuz - 0 derece)
        {11.9388636, 7.4114914},

        // === RED ALLIANCE WALL (TOWER) ===
        // ID 13: Tower, Red (on yuz - 180 derece)
        {16.499332, 7.391908},
        // ID 14: Tower, Red (on yuz - 180 derece)
        {16.499332, 6.960108},
        // ID 15: Tower, Red (on yuz - 180 derece)
        {16.4989764, 4.3124882},
        // ID 16: Tower, Red (on yuz - 180 derece)
        {16.4989764, 3.8806882},

        // === BLUE ALLIANCE TARAFI ===
        // ID 17: Outpost, Blue (arka yuz - 0 derece)
        {4.6490636, 0.6312154},
        // ID 18: Hub, Blue (sol yuz - -90 derece)
        {4.6115986, 3.417951},
        // ID 19: Hub, Blue (arka yuz - 0 derece, dis)
        {5.2151534, 3.6657534},
        // ID 20: Hub, Blue (arka yuz - 0 derece, dis)
        {5.2151534, 4.0213534},
        // ID 21: Hub, Blue (sag yuz - +90 derece, dis)
        {4.6115986, 4.6247558},
        // ID 22: Trench, Blue (arka yuz - 0 derece)
        {4.6490636, 7.4114914},
        // ID 23: Trench, Blue (on yuz - 180 derece)
        {4.574159, 7.4114914},
        // ID 24: Hub, Blue (sag yuz - +90 derece)
        {4.2559986, 4.6247558},
        // ID 25: Hub, Blue (on yuz - 180 derece)
        {4.007866, 4.3769534},
        // ID 26: Hub, Blue (on yuz - 180 derece)
        {4.007866, 4.0213534},
        // ID 27: Hub, Blue (sol yuz - -90 derece)
        {4.2559986, 3.417951},
        // ID 28: Outpost, Blue (on yuz - 180 derece)
        {4.574159, 0.6312154},

        // === BLUE ALLIANCE WALL (TOWER) ===
        // ID 29: Tower, Blue (arka yuz - 0 derece)
        {0.0136906, 0.6507734},
        // ID 30: Tower, Blue (arka yuz - 0 derece)
        {0.0136906, 1.0825734},
        // ID 31: Tower, Blue (arka yuz - 0 derece)
        {0.0140462, 3.7301932},
        // ID 32: Tower, Blue (arka yuz - 0 derece)
        {0.0140462, 4.1619932},
    };

    // --- AprilTag YONLERI (derece, field frame) ---
    private static final double[] TAG_ROTATIONS_DEG = {
        0.0,    // 0 - bos
        180.0,  // 1
        90.0,   // 2
        180.0,  // 3
        180.0,  // 4
        -90.0,  // 5
        180.0,  // 6
        0.0,    // 7
        -90.0,  // 8
        0.0,    // 9
        0.0,    // 10
        90.0,   // 11
        0.0,    // 12
        180.0,  // 13
        180.0,  // 14
        180.0,  // 15
        180.0,  // 16
        0.0,    // 17
        -90.0,  // 18
        0.0,    // 19
        0.0,    // 20
        90.0,   // 21
        0.0,    // 22
        180.0,  // 23
        90.0,   // 24
        180.0,  // 25
        180.0,  // 26
        -90.0,  // 27
        180.0,  // 28
        0.0,    // 29
        0.0,    // 30
        0.0,    // 31
        0.0     // 32
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

    /** Vision seed'i sifirla - sonraki tag gorunce yeniden resetPose yapar */
    public void resetVisionSeed() {
        visionSeeded = false;
        lastAcceptedPose = null;
        lastAcceptedTimestampSeconds = -1.0;
        SmartDashboard.putString("Vision/Status", "SEED RESET - bekleniyor...");
    }

    /** Hizalama aktifken true yap → resetPose devre disi, sadece soft fusion */
    public void setAlignmentActive(boolean active) {
        this.alignmentActive = active;
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

            // 3) Limelight canli durum
            boolean hasTag = LimelightHelpers.getTV(limelightName);
            SmartDashboard.putBoolean("Limelight/HasTarget", hasTag);

            // --- LIMELIGHT DURUM PANELI ---
            // MegaTag2 ile tag sayisi ve mesafe al
            PoseEstimate llEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            int tagCount = (llEstimate != null && llEstimate.tagCount > 0) ? llEstimate.tagCount : 0;
            double avgDist = (llEstimate != null && llEstimate.tagCount > 0) ? llEstimate.avgTagDist : -1;

            // Limelight NetworkTables'dan FPS ve pipeline
            NetworkTable rawLL = ntInst.getTable("limelight");
            double fps = rawLL.getEntry("fps").getDouble(0);
            double pipeline = rawLL.getEntry("getpipe").getDouble(-1);
            double tl = rawLL.getEntry("tl").getDouble(0);  // pipeline latency
            double cl = rawLL.getEntry("cl").getDouble(0);  // capture latency

            llTagCount.set(tagCount);
            llTagDist.set(avgDist > 0 ? round2(avgDist) : -1);
            llFPS.set(Math.round(fps));
            llPipeline.set(pipeline);
            llLatency.set(round2(tl + cl));

            if (!hasTag && fps < 1) {
                llStatus.set("BAGLANTI YOK!");
            } else if (!hasTag) {
                llStatus.set("TAG YOK - " + Math.round(fps) + " FPS");
            } else if (tagCount >= 2) {
                llStatus.set("MUKEMMEL - " + tagCount + " tag, " + round2(avgDist) + "m");
            } else {
                llStatus.set("OK - 1 tag, " + round2(avgDist) + "m");
            }

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
        // VISION FUZYONU - vision kapali degilse HERZAMAN calisir
        // Disabled, teleop, otonom → FARK ETMEZ, tag goruyorsa DUZELT
        // ==================================================================
        if (!enabled) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "DISABLED");
            }
            return;
        }

        // ==================================================================
        // HEADING → LIMELIGHT (MegaTag2 icin KRITIK)
        // Seed ONCESI: MT1 heading varsa onu gonder (gyro yanlis olabilir)
        // Seed SONRASI: Odometry heading gonder (gyro+vision duzeltilmis)
        // ==================================================================
        PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        double yawDeg;
        if (!visionSeeded && mt1Estimate != null && mt1Estimate.pose != null && mt1Estimate.tagCount >= 1) {
            // Seed oncesi: MT1 heading'i gyro'dan daha guvenilir (capraz baslatma!)
            yawDeg = mt1Estimate.pose.getRotation().getDegrees();
        } else {
            // Seed sonrasi: odometry heading (vision ile duzeltilmis)
            yawDeg = currentPose.getRotation().getDegrees();
        }
        double yawRateDps = Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(limelightName, yawDeg, yawRateDps, 0, 0, 0, 0);

        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putBoolean("Vision/LL_HasTarget", LimelightHelpers.getTV(limelightName));
            SmartDashboard.putNumber("Vision/LL_Yaw", round2(yawDeg));
        }

        // Cok hizli donus filtresi
        double omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
        if (Math.abs(omega) > MAX_ANGULAR_VELOCITY_RAD_PER_SEC) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "Skip:Spinning");
            }
            return;
        }

        // ==================================================================
        // POSE TAHMINI AL
        // ==================================================================
        PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // Temel rejection
        if (mt2Estimate == null || mt2Estimate.pose == null) return;
        if (mt2Estimate.tagCount <= 0) return;
        if (mt2Estimate.timestampSeconds <= 0) return;
        if (mt2Estimate.avgTagDist > MAX_AVG_TAG_DISTANCE_METERS) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "Skip:TooFar");
            }
            return;
        }

        // ==================================================================
        // POSE FARKI VE HIZ HESABI
        // ==================================================================
        // MT1 varsa ve 2+ tag ise heading dahil kullan, yoksa MT2 XY + mevcut heading
        Pose2d visionPose;
        boolean mt1HeadingUsed = false;
        if (mt1Estimate != null && mt1Estimate.pose != null && mt1Estimate.tagCount >= 2 && mt1Estimate.avgTagDist < 4.0) {
            // 2+ tag MT1: hem XY hem heading guvenilir
            visionPose = mt1Estimate.pose;
            mt1HeadingUsed = true;
        } else {
            // MT2 XY + mevcut heading (tek tag heading guvenilmez)
            visionPose = mt2Estimate.pose;
        }

        // Saha siniri
        if (visionPose.getX() < -0.5 || visionPose.getX() > FIELD_LENGTH + 0.5
                || visionPose.getY() < -0.5 || visionPose.getY() > FIELD_WIDTH + 0.5) {
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "Skip:OutOfField");
            }
            return;
        }

        double poseDiffMeters = visionPose.getTranslation()
            .getDistance(currentPose.getTranslation());
        double linearSpeedMps = Math.hypot(
            drivetrain.getState().Speeds.vxMetersPerSecond,
            drivetrain.getState().Speeds.vyMetersPerSecond
        );

        // ==================================================================
        // 1) ILK SEED - Tag gorur gormez HEMEN al, bekleme yok
        //    MT1 2+ tag = XY+heading, MT1 1 tag = XY+heading (MT1 tercih)
        //    MT2 = XY + gyro heading (son care)
        // ==================================================================
        if (!visionSeeded) {
            Pose2d seedPose;
            if (mt1Estimate != null && mt1Estimate.pose != null && mt1Estimate.tagCount >= 1) {
                // MT1 varsa HER ZAMAN tercih (heading gyro'dan bagimsiz)
                seedPose = mt1Estimate.pose;
            } else {
                seedPose = mt2Estimate.pose;
            }
            drivetrain.resetPose(seedPose);
            drivetrain.markVisionSeeded();
            visionSeeded = true;
            lastAcceptedPose = visionPose;
            lastAcceptedTimestampSeconds = mt2Estimate.timestampSeconds;
            visionUpdateCount++;
            SmartDashboard.putString("Vision/Status", "SEEDED! tags=" + mt2Estimate.tagCount);
            return;
        }

        // ==================================================================
        // 2) BUYUK FARK → RESETPOSE (aninda duzelt)
        //    Tag gorup konumu duzeltememek OLMAZ.
        // ==================================================================
        if (poseDiffMeters > 1.0 && mt2Estimate.tagCount >= 2) {
            Pose2d resetPose = mt1HeadingUsed ? visionPose
                : new Pose2d(visionPose.getTranslation(), currentPose.getRotation());
            drivetrain.resetPose(resetPose);
            lastAcceptedPose = visionPose;
            lastAcceptedTimestampSeconds = mt2Estimate.timestampSeconds;
            visionUpdateCount++;
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "HARD-RESET! " + round2(poseDiffMeters) + "m");
            }
            return;
        }

        if (poseDiffMeters > 0.5 && mt2Estimate.tagCount >= 1 && linearSpeedMps < 1.0) {
            Pose2d resetPose = (mt1Estimate != null && mt1Estimate.pose != null && mt1Estimate.tagCount >= 1)
                ? mt1Estimate.pose : mt2Estimate.pose;
            drivetrain.resetPose(resetPose);
            lastAcceptedPose = visionPose;
            lastAcceptedTimestampSeconds = mt2Estimate.timestampSeconds;
            visionUpdateCount++;
            if (loopCount % DASHBOARD_INTERVAL == 0) {
                SmartDashboard.putString("Vision/Status", "RESET! " + round2(poseDiffMeters) + "m");
            }
            return;
        }

        // ==================================================================
        // 3) NORMAL FUSION — fark kucuk, Kalman filtre ile ince ayar
        // ==================================================================
        lastAcceptedPose = visionPose;
        lastAcceptedTimestampSeconds = mt2Estimate.timestampSeconds;

        double xyStdDev = XY_STDDEV_COEFFICIENT * Math.pow(mt2Estimate.avgTagDist, 2.0) / mt2Estimate.tagCount;
        xyStdDev = Math.max(xyStdDev, 0.02);

        double headingStdDev = HEADING_STDDEV_SINGLE_TAG;
        if (mt1HeadingUsed) {
            headingStdDev = HEADING_STDDEV_MULTI_TAG;
        }

        drivetrain.addVisionMeasurement(visionPose, mt2Estimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev));
        visionUpdateCount++;

        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Vision/Status",
                "FUSED(t=" + mt2Estimate.tagCount
                + " d=" + round2(mt2Estimate.avgTagDist)
                + " xy=" + round2(xyStdDev)
                + (mt1HeadingUsed ? " +HDG" : "")
                + ")");
            SmartDashboard.putNumber("Vision/Tags", mt2Estimate.tagCount);
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
     * Ilk seed veya buyuk duzeltmede kullanilacak pose.
     * XY her zaman MT2'den, heading ise sadece guvenilir MT1 varsa oradan gelir.
     */
    private Pose2d buildSeedPose(Pose2d xyPose, PoseEstimate mt1Estimate, Pose2d currentPose) {
        if (xyPose == null) {
            return currentPose;
        }

        if (hasReliableHeadingEstimate(mt1Estimate)) {
            return new Pose2d(xyPose.getTranslation(), mt1Estimate.pose.getRotation());
        }

        return new Pose2d(xyPose.getTranslation(), currentPose.getRotation());
    }

    /**
     * MT1 heading'ine ne zaman guvenecegimizi belirler.
     * 2+ tag guvenilir kabul edilir.
     * 1 tag heading kapali - aciyi gyro tasir.
     */
    private boolean hasReliableHeadingEstimate(PoseEstimate mt1Estimate) {
        if (mt1Estimate == null || mt1Estimate.pose == null || mt1Estimate.tagCount <= 0) {
            return false;
        }

        return mt1Estimate.tagCount >= 2 && mt1Estimate.avgTagDist < 4.0;
    }

    /**
     * MT1 / MT2 / tek-tag geometric cozumler arasindan, raw fiducial olcumlerine
     * en tutarli olani secer.
     */
    private Pose2d selectBestVisionPose(PoseEstimate mt2Estimate, PoseEstimate mt1Estimate, Pose2d currentPose) {
        if (mt2Estimate == null || mt2Estimate.pose == null) {
            return currentPose;
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

        SmartDashboard.putNumber("Vision/MT2Score", round2(mt2Score));
        SmartDashboard.putNumber("Vision/MT1Score", round2(mt1Score));

        return mt1Score < mt2Score ? mt1Estimate.pose : mt2Estimate.pose;
    }

    private double scorePoseCandidate(Pose2d candidatePose, RawFiducial[] rawFiducials) {
        if (candidatePose == null || rawFiducials == null || rawFiducials.length == 0) {
            return Double.POSITIVE_INFINITY;
        }

        double totalScore = 0.0;
        int used = 0;

        for (RawFiducial fiducial : rawFiducials) {
            if (fiducial == null) continue;
            int tagId = fiducial.id;
            if (tagId < 1 || tagId >= TAG_POSITIONS.length) continue;

            Translation2d tagPos = new Translation2d(TAG_POSITIONS[tagId][0], TAG_POSITIONS[tagId][1]);
            Translation2d toTag = tagPos.minus(candidatePose.getTranslation());
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

    /**
     * Tek tag gorunurken Limelight'ın robot-space target pose'unu kullanarak
     * robotun saha pozisyonunu dogrudan hesaplar.
     * Bu, tek-tag field pose ambiguity'sini azaltir.
     */
    private Pose2d buildSingleTagPoseFromRobotSpace() {
        int tagId = (int) Math.round(LimelightHelpers.getFiducialID(limelightName));
        if (tagId < 1 || tagId >= TAG_POSITIONS.length) {
            return null;
        }

        Pose2d targetPoseRobotSpace = LimelightHelpers.toPose2D(
            LimelightHelpers.getTargetPose_RobotSpace(limelightName));
        if (targetPoseRobotSpace == null
                || targetPoseRobotSpace.getTranslation().getNorm() < 0.05) {
            return null;
        }

        Pose2d tagFieldPose = new Pose2d(
            TAG_POSITIONS[tagId][0],
            TAG_POSITIONS[tagId][1],
            Rotation2d.fromDegrees(TAG_ROTATIONS_DEG[tagId]));

        Transform2d robotToTag = new Transform2d(
            targetPoseRobotSpace.getTranslation(),
            targetPoseRobotSpace.getRotation());

        return tagFieldPose.transformBy(robotToTag.inverse());
    }

    private static double normalizeDeg(double degrees) {
        return Math.toDegrees(Math.atan2(
            Math.sin(Math.toRadians(degrees)),
            Math.cos(Math.toRadians(degrees))));
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
     * TUMSEK CIKISI HARD RESET - PathPlanner Event Marker ile tetiklenir.
     * Robot tumsekten indikten sonra, AprilTag goruyorsa konumunu aninda duzeltir.
     * resetPose KULLANMAZ (Kalman filtreyi bozar), bunun yerine
     * cok dusuk StdDev ile addVisionMeasurement yapar → ayni etki, guvenli.
     *
     * PathPlanner'da Event Marker ismi: "KonumDuzelt"
     * RobotContainer'da: NamedCommands.registerCommand("KonumDuzelt",
     *     new InstantCommand(() -> vision.forceVisionUpdate()));
     */
    public void forceVisionUpdate() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.pose == null || estimate.tagCount <= 0) {
            SmartDashboard.putString("Vision/Status", "FORCE:NoTag!");
            return;
        }
        if (estimate.avgTagDist > MAX_AVG_TAG_DISTANCE_METERS) {
            SmartDashboard.putString("Vision/Status", "FORCE:TooFar!");
            return;
        }

        // Heading: MegaTag1 2+ tag varsa heading de duzelt
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        double hdgStdDev = 999999;
        if (mt1 != null && mt1.tagCount >= 2) {
            hdgStdDev = 0.05;  // Force modda heading'e ekstra guven
        }

        // Agresif vision update (resetPose degil!)
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds,
            VecBuilder.fill(0.01, 0.01, hdgStdDev));
        visionUpdateCount++;
        SmartDashboard.putString("Vision/Status",
            "FORCE-SYNC(tags=" + estimate.tagCount + " dist=" + round2(estimate.avgTagDist) + "m)");
    }

    /**
     * Vision ile odometry en az 1 kez guncellendi mi?
     * ShootCommand bunu kontrol eder - false ise odometry mesafesine guvenme.
     */
    public boolean hasBeenSeeded() {
        return visionSeeded;
    }

    /**
     * Toplam vision update sayisini dondurur.
     * VisionCorrectCommand bunu kullanarak yeni update gelip gelmedigini takip eder.
     */
    public int getVisionUpdateCount() {
        return visionUpdateCount;
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
