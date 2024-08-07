package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class IntakeCommand extends Command {
    private final IntakeSystem intakeSystem;

    public IntakeCommand(IntakeSystem intakeSystem) {
        this.intakeSystem = intakeSystem;

        addRequirements(intakeSystem);
    }

    public void initialize() {
    }

    public void execute() {
        intakeSystem.in();
    }

    public void end(boolean interrupted) {
        intakeSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
