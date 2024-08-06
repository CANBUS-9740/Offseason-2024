package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSystem;

public class ArmDownCommand extends Command {
    private ArmSystem armSystem;

    public ArmDownCommand(ArmSystem armSystem) {
        this.armSystem = armSystem;

        addRequirements(armSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armSystem.moveDown();
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
