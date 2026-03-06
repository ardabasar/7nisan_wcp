package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SHOOTER SUBSYSTEM - WCP BIREBIR KOPYA
 *
 * 3x Kraken X60, VelocityVoltage PID ile RPM kontrolu.
 *   Motor 1 (CAN 9):  CounterClockwise_Positive (+1 yon)
 *   Motor 2 (CAN 10): Clockwise_Positive (-1 yon)
 *   Motor 3 (CAN 11): Clockwise_Positive (-1 yon)
 *   Coast mode, PeakReverseVoltage = 0
 *
 * PID: kP=0.5, kI=2, kD=0, kV=12/freeSpeedRPS
 * Current: 120A stator, 70A supply
 */
public class ShooterSubsystem extends SubsystemBase {

    public static final int MOTOR_1_CAN_ID = 9;
    public static final int MOTOR_2_CAN_ID = 10;
    public static final int MOTOR_3_CAN_ID = 11;
    public static final String CAN_BUS     = "rio";

    // Kraken X60 free speed = 6000 RPM = 100 RPS
    private static final double KRAKEN_FREE_SPEED_RPS = 100.0;

    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX motor3;
    private final List<TalonFX> motors;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ShooterSubsystem() {
        motor1 = new TalonFX(MOTOR_1_CAN_ID, CAN_BUS);
        motor2 = new TalonFX(MOTOR_2_CAN_ID, CAN_BUS);
        motor3 = new TalonFX(MOTOR_3_CAN_ID, CAN_BUS);
        motors = List.of(motor1, motor2, motor3);

        configureMotor(motor1, InvertedValue.CounterClockwise_Positive);
        configureMotor(motor2, InvertedValue.Clockwise_Positive);
        configureMotor(motor3, InvertedValue.Clockwise_Positive);

        stop();
    }

    /** WCP configureMotor birebir kopya */
    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / KRAKEN_FREE_SPEED_RPS)
            );

        motor.getConfigurator().apply(config);
    }

    /** WCP setRPM birebir kopya - VelocityVoltage ile RPM hedefi */
    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
            motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }
    }

    /** WCP setPercentOutput birebir kopya */
    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
            motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
        }
    }

    /** WCP stop birebir kopya */
    public void stop() {
        setPercentOutput(0.0);
    }

    /** WCP isVelocityWithinTolerance birebir kopya */
    public boolean isVelocityWithinTolerance() {
        return isVelocityWithinTolerance(kVelocityTolerance.in(RPM));
    }

    /**
     * Tum shooter motorlari hedef RPM'e (mutlak deger olarak) belirtilen tolerans
     * dahilinde mi?
     */
    public boolean isVelocityWithinTolerance(double toleranceRpm) {
        final double targetRpmAbs = Math.abs(velocityRequest.getVelocityMeasure().in(RPM));
        return motors.stream().allMatch(motor -> {
            final double currentRpmAbs = Math.abs(motor.getVelocity().getValue().in(RPM));
            return Math.abs(currentRpmAbs - targetRpmAbs) <= toleranceRpm;
        });
    }

    /**
     * En yavas shooter motorunun RPM'i, hedefin belirli bir oranina ulasti mi?
     * Bu, toleransa tam giremeyen ancak pratikte atis yapabilecek durumlarda
     * feeder/hopper'i guvenli sekilde baslatmak icin kullanilir.
     */
    public boolean isVelocityAboveRatio(double ratio) {
        final double targetRpmAbs = Math.abs(getTargetRPM());
        if (targetRpmAbs < 1.0) return false;
        final double minMotorRpmAbs = motors.stream()
            .mapToDouble(motor -> Math.abs(motor.getVelocity().getValue().in(RPM)))
            .min()
            .orElse(0.0);
        return minMotorRpmAbs >= targetRpmAbs * ratio;
    }

    public double getMinMotorRPMAbs() {
        return motors.stream()
            .mapToDouble(motor -> Math.abs(motor.getVelocity().getValue().in(RPM)))
            .min()
            .orElse(0.0);
    }

    public double getAverageMotorRPMAbs() {
        return motors.stream()
            .mapToDouble(motor -> Math.abs(motor.getVelocity().getValue().in(RPM)))
            .average()
            .orElse(0.0);
    }

    public boolean isRunning() {
        return velocityRequest.getVelocityMeasure().in(RPM) > 0;
    }

    public double getTargetRPM() {
        return velocityRequest.getVelocityMeasure().in(RPM);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Motor1 RPM", motor1.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Shooter/Motor2 RPM", motor2.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Shooter/Motor3 RPM", motor3.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Shooter/Target RPM", getTargetRPM());
        SmartDashboard.putNumber("Shooter/Min RPM Abs", getMinMotorRPMAbs());
        SmartDashboard.putNumber("Shooter/Avg RPM Abs", getAverageMotorRPMAbs());
        SmartDashboard.putBoolean("Shooter/AtSpeed", isVelocityWithinTolerance());
    }
}
