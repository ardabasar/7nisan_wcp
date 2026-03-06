package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * INTAKE ARM SUBSYSTEM - 2026 REBUILT
 * ============================================================================
 * Intake kolunu yukari/asagi hareket ettiren motor.
 * Kraken X60 dahili encoder ile pozisyon takibi yapar.
 *
 * Donanim:
 *   - Motor: CAN 12 (rio bus)
 *   - Brake mode (kol dusmemeli)
 *   - Dahili encoder (Kraken kendi ici, harici encoder YOK)
 *
 * Kontrol:
 *   - Baslangicta encoder 0'a set edilir
 *   - Tusa basilinca hedef pozisyona gider (DutyCycle ile)
 *
 * Pozisyon degerleri sahada kalibre edilmistir:
 *   STOW_POSITION  = -0.82 (kapali/yukari)
 *   INTAKE_POSITION =  6.65 (acik/asagi)
 * ============================================================================
 */
public class IntakeArmSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID
    // ========================================================================
    public static final int MOTOR_CAN_ID = 12;
    public static final String CAN_BUS   = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================

    /**
     * Intake kolunun hedef pozisyonlari (rotor rotation cinsinden).
     * Sahada Elastic ile olculmustur.
     *
     *   STOW_POSITION = -0.82  → kol kapali/yukari (baslangic)
     *   INTAKE_POSITION = 6.65 → kol acik/asagi (top alma pozisyonu)
     */
    public static final double STOW_POSITION   = -0.82;
    public static final double INTAKE_POSITION  =  6.65;

    /** Kol hareket hizi (DutyCycle) - her iki yonde 0.25 */
    public static double ARM_SPEED = 0.25;

    /** Kol pozisyon toleransi (rotasyon cinsinden) */
    public static final double POSITION_TOLERANCE = 0.5;

    /** Stator akim limiti */
    private static final double STATOR_CURRENT_LIMIT = 40.0;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // ========================================================================
    // TELEMETRI
    // ========================================================================
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Current> currentSignal;

    // ========================================================================
    // STATE
    // ========================================================================
    private double targetSpeed = 0.0;
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public IntakeArmSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);
        configureMotor();

        positionSignal = motor.getRotorPosition();
        currentSignal  = motor.getStatorCurrent();

        // Baslangicta encoder sifirla
        motor.setPosition(0.0);

        stop();

        System.out.println("[IntakeArm] Initialized - CAN " + MOTOR_CAN_ID
            + " | Encoder = 0 | INTAKE_POSITION = " + INTAKE_POSITION);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        motor.getConfigurator().apply(config);
    }

    // ========================================================================
    // KONTROL
    // ========================================================================

    /** Kolu belirtilen hizda hareket ettirir. */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Kolu belirtilen pozisyona dogru hareket ettirir.
     *  0.0 = stow (kapali), INTAKE_POSITION = acik. */
    public void goToPosition(double targetPos) {
        double current = getPosition();
        if (Math.abs(current - targetPos) < POSITION_TOLERANCE) {
            stop();
        } else if (current < targetPos) {
            setSpeed(ARM_SPEED);
        } else {
            setSpeed(-ARM_SPEED);
        }
    }

    /** Kolu durdurur (Brake mode tutar). */
    public void stop() {
        targetSpeed = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    /** Encoder'i sifirlar (su anki pozisyonu 0 olarak ayarlar). */
    public void resetEncoder() {
        motor.setPosition(0.0);
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    /** Suanki pozisyon (rotor rotation) */
    public double getPosition() {
        return positionSignal.refresh().getValueAsDouble();
    }

    /** Hedef pozisyona ulasti mi? */
    public boolean atPosition(double targetPos) {
        return Math.abs(getPosition() - targetPos) < POSITION_TOLERANCE;
    }

    /** Motor aktif mi? */
    public boolean isActive() {
        return Math.abs(targetSpeed) > 0.01;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("IntakeArm/Position", Math.round(getPosition() * 100.0) / 100.0);
        SmartDashboard.putNumber("IntakeArm/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("IntakeArm/Speed", targetSpeed);
        SmartDashboard.putBoolean("IntakeArm/Active", isActive());
    }
}
