package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSystem;

public class ArmMoveToFloorCommand extends Command {
    private final ArmSystem armSystem;

    public ArmMoveToFloorCommand(ArmSystem armSystem) {
        this.armSystem = armSystem;

        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        armSystem.moveToPosition(RobotMap.ARM_FLOOR_ANGLE);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return armSystem.researchATargetAngle(RobotMap.ARM_FLOOR_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        armSystem.stop();
    }
}
