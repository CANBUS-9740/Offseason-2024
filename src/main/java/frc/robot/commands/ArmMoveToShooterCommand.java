package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSystem;

public class ArmMoveToShooterCommand extends Command {
    private final ArmSystem armSystem;
    private final PIDController pidController;

    public ArmMoveToShooterCommand(ArmSystem armSystem) {
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
        double output = pidController.calculate(armSystem.getAbsEncoderPositionDegrees(),  RobotMap.ARM_SHOOTER_ANGLE);

        armSystem.move(output);
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