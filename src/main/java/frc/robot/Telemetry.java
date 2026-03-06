package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * WPILib Field2d - Elastic dashboard'da robot + swerve modulleri gosterir.
     * Robot hem X/Y hem de rotasyon (heading) ile birlikte ciziyor.
     * Her swerve modulu de kendi yonuyle ayri obje olarak gorunur.
     */
    private final Field2d m_field = new Field2d();
    private final FieldObject2d[] m_swerveModules = new FieldObject2d[] {
        m_field.getObject("FrontLeft"),
        m_field.getObject("FrontRight"),
        m_field.getObject("BackLeft"),
        m_field.getObject("BackRight"),
    };

    /**
     * Swerve modul pozisyonlari (robot merkezine gore, metre cinsinden).
     * TunerConstants'tan: 11.4 inch = 0.28956 m
     */
    private static final Translation2d[] MODULE_OFFSETS = new Translation2d[] {
        new Translation2d( 0.28956,  0.28956), // Front Left
        new Translation2d( 0.28956, -0.28956), // Front Right
        new Translation2d(-0.28956,  0.28956), // Back Left
        new Translation2d(-0.28956, -0.28956), // Back Right
    };

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        // Field2d'yi SmartDashboard'a ekle (Elastic bunu otomatik gosterebilir)
        SmartDashboard.putData("Field", m_field);

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }

        /* Elastic Dashboard SwerveDrive widget - ustten gorunus sase animasyonu.
         * Modul acilari robot-relative (robotun kendi eksenine gore).
         * Robot Angle = robotun saha uzerindeki heading'i (field-relative).
         * Elastic bu ikisini birlestirerek field-relative gorunum saglar.
         * NOT: Elastic SwerveDrive widget'i acilari DERECE cinsinden ister,
         * radian degil! getRadians() yerine getDegrees() kullanilmali.
         */
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle",
                    () -> m_lastModuleAnglesDeg[0], null);
                builder.addDoubleProperty("Front Left Velocity",
                    () -> m_cachedStates[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle",
                    () -> m_lastModuleAnglesDeg[1], null);
                builder.addDoubleProperty("Front Right Velocity",
                    () -> m_cachedStates[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle",
                    () -> m_lastModuleAnglesDeg[2], null);
                builder.addDoubleProperty("Back Left Velocity",
                    () -> m_cachedStates[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle",
                    () -> m_lastModuleAnglesDeg[3], null);
                builder.addDoubleProperty("Back Right Velocity",
                    () -> m_cachedStates[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle",
                    () -> -Math.toDegrees(m_cachedRobotAngle), null);
            }
        });
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /* Son swerve state - Elastic SwerveDrive widget icin cache */
    private volatile SwerveModuleState[] m_cachedStates = new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(),
        new SwerveModuleState(), new SwerveModuleState(),
    };
    private volatile double m_cachedRobotAngle = 0.0;

    /**
     * Son bilinen modul acilari (derece).
     * Robot durdugunda (hiz ~0) swerve modul acisi belirsiz olur ve
     * rastgele ziplayarak widget'in cildirmis gibi donmesine neden olur.
     * Cozum: hiz dusukken son bilinen aciyi koru, guncelleme.
     */
    private final double[] m_lastModuleAnglesDeg = new double[4];
    private static final double SPEED_DEADBAND = 0.05; // m/s altinda aci guncellenmez

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Cache state for Elastic SwerveDrive widget.
         * Hiz cok dusukse aciyi GUNCELLEME - son bilinen aciyi koru.
         * Bu, widget'in robot dururken 360 donmesini engeller. */
        for (int i = 0; i < 4; i++) {
            if (Math.abs(state.ModuleStates[i].speedMetersPerSecond) > SPEED_DEADBAND) {
                m_lastModuleAnglesDeg[i] = state.ModuleStates[i].angle.getDegrees();
            }
        }
        m_cachedStates = state.ModuleStates;
        m_cachedRobotAngle = state.Pose.getRotation().getRadians();

        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* WPILib Field2d - robot pose + swerve modul pozisyonlari */
        m_field.setRobotPose(state.Pose);

        // Her swerve modulunun saha uzerindeki gercek pozisyonunu hesapla
        Rotation2d robotAngle = state.Pose.getRotation();
        Translation2d robotTranslation = state.Pose.getTranslation();
        for (int i = 0; i < 4; ++i) {
            // Modul offset'ini robotun acisina gore dondur ve robot pozisyonuna ekle
            Translation2d moduleFieldPos = robotTranslation.plus(
                MODULE_OFFSETS[i].rotateBy(robotAngle)
            );
            // Modulun saha uzerindeki acisi = robot acisi + modul steer acisi
            Rotation2d moduleFieldAngle = robotAngle.plus(state.ModuleStates[i].angle);
            m_swerveModules[i].setPose(new Pose2d(moduleFieldPos, moduleFieldAngle));
        }

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }
}
