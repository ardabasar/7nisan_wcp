package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * FEEDER SUBSYSTEM - WCP BIREBIR KOPYA
 *
 * CAN 15 (rio bus), Coast mode
 * VelocityVoltage PID ile 5000 RPM sabit hiz.
 * PID: kP=1, kI=0, kD=0, kV=12/100
 * Current: 120A stator, 50A supply
 */
public class FeederSubsystem extends SubsystemBase {

    public static final int MOTOR_CAN_ID = 15;
    public static final String CAN_BUS   = "rio";

    /** WCP Feeder.Speed.FEED = 5000 RPM */
    public static final double FEED_RPM = 5000.0;

    // Kraken X60 free speed RPS
    private static final double KRAKEN_FREE_SPEED_RPS = 100.0;

    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public FeederSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);

        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KRAKEN_FREE_SPEED_RPS)
            );

        motor.getConfigurator().apply(config);
        stop();
    }

    /** WCP set(Speed.FEED) birebir kopya - 5000 RPM */
    public void feed() {
        motor.setControl(velocityRequest.withVelocity(RPM.of(FEED_RPM)));
    }

    /** WCP setPercentOutput birebir kopya */
    public void setPercentOutput(double percentOutput) {
        motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
    }

    /** WCP stop = setPercentOutput(0) */
    public void stop() {
        setPercentOutput(0.0);
    }

    public boolean isRunning() {
        return Math.abs(motor.getVelocity().getValue().in(RPM)) > 50;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/RPM", motor.getVelocity().getValue().in(RPM));
        SmartDashboard.putBoolean("Feeder/Running", isRunning());
    }
}
