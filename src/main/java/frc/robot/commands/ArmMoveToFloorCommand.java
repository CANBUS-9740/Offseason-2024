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
        pidController = new PIDController(RobotMap.ARM_PID_P, RobotMap.ARM_PID_I, RobotMap.ARM_PID_D);

        addRequirements(armSystem);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double power = pidController.calculate(armSystem.getAbsEncoderPositionDegrees(), RobotMap.ARM_FLOOR_ANGLE);

        armSystem.move(power);
    }

    @Override
    public boolean isFinished() {
        return armSystem.reachedATargetAngle(RobotMap.ARM_FLOOR_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        armSystem.stop();
    }
}
