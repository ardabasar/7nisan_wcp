package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * INTAKE ARM SUBSYSTEM - 2026 REBUILT
 * ============================================================================
 * Intake kolunu yukari/asagi hareket ettiren motor.
 *
 * Donanim:
 *   - Motor: CAN 12 (rio bus)
 *   - Brake mode (kol dusmemeli)
 *
 * Kontrol:
 *   - A tusuna basili tut -> kol acar (+0.25 hiz)
 *   - LB tusuna basili tut -> kol kapatir (-0.25 hiz)
 *   - Tus birakilinca durur (Brake mode tutar)
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

    /** Kol hareket hizi (DutyCycle) - her iki yonde 0.25 */
    public static double ARM_SPEED = 0.25;

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

        currentSignal = motor.getStatorCurrent();

        stop();

        System.out.println("[IntakeArm] Initialized - CAN " + MOTOR_CAN_ID);
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

    /** Kolu belirtilen hizda hareket ettirir. */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Kolu durdurur (Brake mode tutar). */
    public void stop() {
        targetSpeed = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

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

        SmartDashboard.putNumber("IntakeArm/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("IntakeArm/Speed", targetSpeed);
        SmartDashboard.putBoolean("IntakeArm/Active", isActive());
    }
}
