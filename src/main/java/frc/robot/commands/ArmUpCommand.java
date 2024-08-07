package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSystem;

public class ArmUpCommand extends Command {
    private final ArmSystem armSystem;

    public ArmUpCommand(ArmSystem armSystem) {
        this.armSystem = armSystem;

        addRequirements(armSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armSystem.moveUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSystem.stop();
    }
}