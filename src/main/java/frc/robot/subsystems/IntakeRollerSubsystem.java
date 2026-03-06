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
    public static final double ROLLER_SPEED = -0.5;

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
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
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

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("IntakeRoller/VelocityRPS", Math.round(getVelocityRPS() * 100.0) / 100.0);
        SmartDashboard.putNumber("IntakeRoller/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("IntakeRoller/Speed", targetSpeed);
        SmartDashboard.putBoolean("IntakeRoller/Active", isActive());
    }
}
