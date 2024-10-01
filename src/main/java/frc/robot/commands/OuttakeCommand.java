package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class OuttakeCommand extends Command {
    private final IntakeSystem intakeSystem;

    public OuttakeCommand(IntakeSystem intakeSystem) {
        this.intakeSystem = intakeSystem;

        addRequirements(intakeSystem);
    }

    public void initialize() {
    }

    public void execute() {
        intakeSystem.out();
    }

    public void end(boolean interrupted) {
        intakeSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
