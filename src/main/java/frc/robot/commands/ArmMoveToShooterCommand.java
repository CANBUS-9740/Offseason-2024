package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSystem;

public class ArmMoveToShooterCommand extends Command {
    private final ArmSystem armSystem;

    public ArmMoveToShooterCommand(ArmSystem armSystem) {
        this.armSystem = armSystem;

        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        armSystem.moveToPosition(RobotMap.ARM_SHOOTER_ANGLE);
    }

    @Override
    public void execute() {
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