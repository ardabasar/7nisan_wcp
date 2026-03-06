package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * ============================================================================
 * INTAKE COMMAND - 2026 REBUILT
 * ============================================================================
 * A tusuna basili tut:
 *   - Arm (CAN 12) +0.25 hizda acar
 *   - Roller (CAN 13) 0.5 hizda calisir
 *   - Birakinca ikisi de durur
 * ============================================================================
 */
public class IntakeCommand extends Command {

    private final IntakeArmSubsystem arm;
    private final IntakeRollerSubsystem roller;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    public IntakeCommand(IntakeArmSubsystem arm, IntakeRollerSubsystem roller) {
        this.arm = arm;
        this.roller = roller;
        addRequirements(arm, roller);
    }

    @Override
    public void initialize() {
        loopCount = 0;
    }

    @Override
    public void execute() {
        loopCount++;

        // Basili tutuldugu surece: arm acar, roller doner
        arm.setSpeed(IntakeArmSubsystem.ARM_SPEED);  // +0.25
        roller.run();                                  // 0.5

        // Dashboard
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putNumber("Intake/ArmPos", Math.round(arm.getPosition() * 100.0) / 100.0);
            SmartDashboard.putBoolean("Intake/RollerActive", roller.isActive());
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        roller.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
