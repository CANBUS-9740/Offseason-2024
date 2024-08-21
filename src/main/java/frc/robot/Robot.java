package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ArmMoveToFloorCommand;
import frc.robot.commands.ArmMoveToShooterCommand;
import frc.robot.subsystems.ArmSystem;

public class Robot extends TimedRobot {
    private XboxController xboxController;
    private ArmSystem armSystem;

    @Override
    public void robotInit() {
        xboxController = new XboxController(0);
        armSystem = new ArmSystem();

        POVButton dPadUp = new POVButton(xboxController, 0);
        POVButton dPadDown = new POVButton(xboxController, 180);

        dPadUp.whileTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.whileTrue(new ArmMoveToFloorCommand(armSystem));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }
}
