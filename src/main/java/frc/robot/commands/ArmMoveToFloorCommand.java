package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSystem;

public class ArmMoveToFloorCommand extends Command {
    private final ArmSystem armSystem;
    private final PIDController pidController;

    public ArmMoveToFloorCommand(ArmSystem armSystem) {
        this.armSystem = armSystem;
        pidController = new PIDController(0.07, 0, 0);

        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double output = pidController.calculate(armSystem.getAbsEncoderPositionDegrees(), RobotMap.ARM_FLOOR_ANGLE);

        armSystem.moveToAngle(output);
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
