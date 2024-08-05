package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSystem;

public class ArmCommand extends Command {
    private final ArmSystem armSystem;
    private final XboxController xboxController;

    public ArmCommand(ArmSystem sub, XboxController xboxController) {
        this.armSystem = sub;
        this.xboxController = xboxController;

        addRequirements(sub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (xboxController.getPOV() == 0) {
            armSystem.moveUp();
        } else if (xboxController.getPOV() == 180) {
            armSystem.moveDown();
        } else {
            armSystem.stop();
        }
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