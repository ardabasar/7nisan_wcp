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

/**
 * ============================================================================
 * HOPPER SUBSYSTEM - 2026 REBUILT
 * ============================================================================
 * Alttaki kayis sistemi - toplari surekli shooter'a besler.
 *
 * Donanim:
 *   - Motor: CAN 14 (rio bus)
 *   - -1 tarafina dogru calisir
 *   - Brake mode
 *
 * Kontrol:
 *   - -0.25 hizda surekli calisir (shooter beslemesi)
 *   - -1 yonunde kayislari calistirir
 * ============================================================================
 */
public class HopperSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID
    // ========================================================================
    public static final int MOTOR_CAN_ID = 14;
    public static final String CAN_BUS   = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================
    /** Hopper calisme hizi (-1 yone dogru, bu yuzden negatif) */
    public static final double HOPPER_SPEED = -1;

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
    private final StatusSignal<AngularVelocity> velocitySignal;
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
    public HopperSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);
        configureMotor();

        velocitySignal = motor.getRotorVelocity();
        currentSignal  = motor.getStatorCurrent();

        stop();
        System.out.println("[Hopper] Initialized - CAN " + MOTOR_CAN_ID);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 38.0;
        motor.getConfigurator().apply(config);
    }

    // ========================================================================
    // KONTROL
    // ========================================================================

    /** Hopper'i varsayilan hizda calistirir (-0.25, -1 yone dogru). */
    public void run() {
        targetSpeed = HOPPER_SPEED;
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Hopper'i belirtilen hizda calistirir. */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Hopper'i ters calistirir. */
    public void reverse() {
        targetSpeed = -HOPPER_SPEED; // +0.25
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Hopper'i durdurur. */
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

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Hopper/VelocityRPS", Math.round(getVelocityRPS() * 100.0) / 100.0);
        SmartDashboard.putNumber("Hopper/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("Hopper/Speed", targetSpeed);
        SmartDashboard.putBoolean("Hopper/Active", isActive());
    }
}
