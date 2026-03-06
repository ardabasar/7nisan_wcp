package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * ============================================================================
 * CLIMB COMMAND - 2026 REBUILT
 * ============================================================================
 * Asma komutu. Iki butonla calisir:
 *   - UP butonu:   +0.25 hizda yukari
 *   - DOWN butonu: -0.25 hizda asagi
 *   - Buton birakilinca durur (Brake mode tutar)
 * ============================================================================
 */
public class ClimbCommand extends Command {

    public enum Direction { UP, DOWN }

    private final ClimbSubsystem climb;
    private final Direction direction;

    public ClimbCommand(ClimbSubsystem climb, Direction direction) {
        this.climb = climb;
        this.direction = direction;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        // Bos - execute'da calisacak
    }

    @Override
    public void execute() {
        switch (direction) {
            case UP:
                climb.climbUp();    // +0.25
                break;
            case DOWN:
                climb.climbDown();  // -0.25
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // whileTrue ile kullanilir
    }
}
