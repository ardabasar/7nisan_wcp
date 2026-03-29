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
 * CLIMB SUBSYSTEM - 2026 REBUILT
 * ============================================================================
 * Asma mekanizmasi.
 * 2 tusu var: birine basinca +0.25 hizda, digerine basinca -0.25 hizda.
 *
 * Donanim:
 *   - Motor: CAN 16 (rio bus)
 *   - Brake mode (asili kalabilmeli)
 *
 * Kontrol:
 *   - Buton 1 (yukari):  +0.25 DutyCycle
 *   - Buton 2 (asagi):   -0.25 DutyCycle
 *   - Buton birakilinca durur (Brake tutar)
 * ============================================================================
 */
public class ClimbSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID
    // ========================================================================
    public static final int MOTOR_CAN_ID = 16;
    public static final String CAN_BUS   = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================
    /** Climb hareket hizi */
    public static final double CLIMB_SPEED = -0.5;

    /** Stator akim limiti */
    private static final double STATOR_CURRENT_LIMIT = 50.0;

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
    public ClimbSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);
        configureMotor();

        positionSignal = motor.getRotorPosition();
        currentSignal  = motor.getStatorCurrent();

        stop();
        System.out.println("[Climb] Initialized - CAN " + MOTOR_CAN_ID);
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

    /** Climb'i yukari calistirir (+0.25). */
    public void climbUp() {
        targetSpeed = -CLIMB_SPEED;
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Climb'i asagi calistirir (-0.25). */
    public void climbDown() {
        targetSpeed = CLIMB_SPEED;
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Climb'i belirtilen hizda calistirir. */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /** Climb'i durdurur (Brake mode tutar). */
    public void stop() {
        targetSpeed = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================
    public double getPosition() { return positionSignal.refresh().getValueAsDouble(); }
    public boolean isActive() { return Math.abs(targetSpeed) > 0.01; }
    public double getTargetSpeed() { return targetSpeed; }

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Climb/Position", Math.round(getPosition() * 100.0) / 100.0);
        SmartDashboard.putNumber("Climb/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("Climb/Speed", targetSpeed);
        SmartDashboard.putBoolean("Climb/Active", isActive());
    }
}
