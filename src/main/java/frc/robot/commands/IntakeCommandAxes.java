package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class IntakeCommandAxes extends Command {
    private IntakeSystem intakeSystem;
    private XboxController xboxController;

    public IntakeCommandAxes(IntakeSystem intakeSystem, XboxController xboxController) {
        this.xboxController = xboxController;
        this.intakeSystem = intakeSystem;

        addRequirements(intakeSystem);
    }

    public void initialize() {
    }

    public void execute() {
        if(MathUtil.isNear(1,xboxController.getRightY(),0.5)) {
            intakeSystem.in();
        } else if (MathUtil.isNear(-1,xboxController.getRightY(),0.5)) {
            intakeSystem.out();
        } else {
            intakeSystem.stop();
        }
    }

    public void end(boolean interrupted) {
        intakeSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
