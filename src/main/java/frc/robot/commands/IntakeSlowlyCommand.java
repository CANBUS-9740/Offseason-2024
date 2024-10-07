package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class IntakeSlowlyCommand extends Command {
    private final IntakeSystem intakeSystem;

    public IntakeSlowlyCommand(IntakeSystem intakeSystem) {
        this.intakeSystem = intakeSystem;

        addRequirements(intakeSystem);
    }

    public void initialize() {
    }

    public void execute() {
        intakeSystem.slowIn();
    }

    public void end(boolean interrupted) {
        intakeSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
