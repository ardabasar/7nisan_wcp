package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Elastic;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * ============================================================================
 * INTAKE ROLLER SUBSYSTEM - 2026 REBUILT
 * ============================================================================
 * Intake kolundaki silindir/roller motoru. Top alimini yapar.
 * IntakeArm (CAN 12) belli pozisyona gelince bu motor devreye girer.
 *
 * Donanim:
 *   - Motor: CAN 13 (rio bus)
 *   - Brake mode
 *
 * Kontrol:
 *   - IntakeArm hedef pozisyona gelince 0.5 hizda calisir
 *   - Top alindiktan sonra durur
 *   - IntakeArm'a BAGIMLI - IntakeCommand tarafindan yonetilir
 *
 * Zorlanma Tespiti:
 *   - Motor aktif + Yuksek akim + Dusuk hiz = SIKISMA
 *   - Elastic'e bildirim gonderir (Turuncu/Yesil/Kirmizi)
 * ============================================================================
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID
    // ========================================================================
    public static final int MOTOR_CAN_ID = 13;
    public static final String CAN_BUS   = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================
    /** Roller calisme hizi */
    public static final double ROLLER_SPEED = -1;

    /** Stator akim limiti */
    private static final double STATOR_CURRENT_LIMIT = 40.0;

    /** Zorlanma/sikisma tespit esikleri */
    private static final double STALL_CURRENT_THRESHOLD = 25.0;  // Amper - bu uzerinde zorlanma
    private static final double STALL_VELOCITY_THRESHOLD = 50.0; // RPM - bu altinda hareket yok
    private static final int STALL_DETECTION_CYCLES = 5;         // Art arda bu kadar cycle = sikisma

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // ========================================================================
    // TELEMETRI
    // ========================================================================
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> currentSignal;

    // ========================================================================
    // STATE
    // ========================================================================
    private double targetSpeed = 0.0;
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // Zorlanma tespit state
    private int stallCounter = 0;
    private boolean isStalled = false;
    private MotorStatus lastStatus = MotorStatus.IDLE;

    /** Motor durum enum - Elastic renkleri icin */
    public enum MotorStatus {
        IDLE,       // Turuncu - Motor durgun
        RUNNING,    // Yesil - Normal calisiyor
        STALLED     // Kirmizi - Zorlanma/sikisma
    }

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public IntakeRollerSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);
        configureMotor();

        velocitySignal = motor.getRotorVelocity();
        currentSignal  = motor.getStatorCurrent();

        stop();
        System.out.println("[IntakeRoller] Initialized - CAN " + MOTOR_CAN_ID);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        motor.getConfigurator().apply(config);
    }

    // ========================================================================
    // KONTROL
    // ========================================================================

    /** Roller'i belirtilen hizda calistirir. */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Roller'i varsayilan hizda calistirir (0.5). */
    public void run() {
        setSpeed(ROLLER_SPEED);
    }

    /** Roller'i ters calistirir. */
    public void reverse() {
        setSpeed(-ROLLER_SPEED);
    }

    /** Roller'i durdurur. */
    public void stop() {
        targetSpeed = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================
    public double getVelocityRPS() { return velocitySignal.refresh().getValueAsDouble(); }
    public boolean isActive() { return Math.abs(targetSpeed) > 0.01; }
    public double getTargetSpeed() { return targetSpeed; }

    /** Motor sikismis mi? */
    public boolean isStalled() {
        return isStalled;
    }

    /** Motor durumunu dondurur (Elastic renk icin) */
    public MotorStatus getMotorStatus() {
        if (isStalled) return MotorStatus.STALLED;
        if (isActive()) return MotorStatus.RUNNING;
        return MotorStatus.IDLE;
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        // Her cycle'da zorlanma kontrolu yap
        checkStallCondition();

        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        double current = currentSignal.refresh().getValueAsDouble();
        double velocityRPS = velocitySignal.refresh().getValueAsDouble();
        double velocityRPM = velocityRPS * 60.0;

        SmartDashboard.putNumber("IntakeRoller/VelocityRPS", Math.round(velocityRPS * 100.0) / 100.0);
        SmartDashboard.putNumber("IntakeRoller/VelocityRPM", Math.round(velocityRPM));
        SmartDashboard.putNumber("IntakeRoller/Current", Math.round(current * 10.0) / 10.0);
        SmartDashboard.putNumber("IntakeRoller/Speed", targetSpeed);
        SmartDashboard.putBoolean("IntakeRoller/Active", isActive());
        SmartDashboard.putBoolean("IntakeRoller/Stalled", isStalled);
        SmartDashboard.putString("IntakeRoller/Status", getMotorStatus().name());
    }

    // ========================================================================
    // ZORLANMA TESPITI
    // ========================================================================
    private void checkStallCondition() {
        double current = currentSignal.refresh().getValueAsDouble();
        double velocityRPS = velocitySignal.refresh().getValueAsDouble();
        double velocityRPM = Math.abs(velocityRPS * 60.0);

        MotorStatus currentStatus;

        // Motor aktif mi?
        if (!isActive()) {
            // Motor durgun - IDLE
            stallCounter = 0;
            isStalled = false;
            currentStatus = MotorStatus.IDLE;
        } else {
            // Motor calisiyor - zorlanma kontrolu
            // Yuksek akim + dusuk hiz = sikisma
            if (current > STALL_CURRENT_THRESHOLD && velocityRPM < STALL_VELOCITY_THRESHOLD) {
                stallCounter++;
                if (stallCounter >= STALL_DETECTION_CYCLES) {
                    isStalled = true;
                    currentStatus = MotorStatus.STALLED;
                } else {
                    currentStatus = MotorStatus.RUNNING;
                }
            } else {
                // Normal calisiyor
                stallCounter = 0;
                isStalled = false;
                currentStatus = MotorStatus.RUNNING;
            }
        }

        // Durum degistiyse Elastic'e bildirim gonder
        if (currentStatus != lastStatus) {
            sendElasticNotification(currentStatus);
            lastStatus = currentStatus;
        }
    }

    private void sendElasticNotification(MotorStatus status) {
        switch (status) {
            case IDLE:
                Elastic.sendNotification(
                    new Elastic.Notification(NotificationLevel.INFO, "IntakeRoller", "Durgun")
                        .withDisplaySeconds(2.0));
                break;
            case RUNNING:
                Elastic.sendNotification(
                    new Elastic.Notification(NotificationLevel.INFO, "IntakeRoller", "Calisiyor")
                        .withDisplaySeconds(2.0));
                break;
            case STALLED:
                // KIRMIZI - Zorlanma/Sikisma - ERROR seviyesi
                // displaySeconds cok yuksek - durum degisince (RUNNING/IDLE) yeni bildirim gelecek
                Elastic.sendNotification(
                    new Elastic.Notification(NotificationLevel.ERROR, "INTAKE ROLLER SIKISTI!", 
                        "Motor zorlanma tespit edildi! Akim yuksek, hareket yok.")
                        .withDisplaySeconds(9999.0));
                break;
        }
    }
}
