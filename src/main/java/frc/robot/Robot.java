package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootOut;
import frc.robot.commands.ShooterTeleop;
import frc.robot.subsystems.ShooterSystem;

public class Robot extends TimedRobot {
    private XboxController xboxController;
    private ShooterSystem shooterSystem;

    @Override
    public void robotInit() {
        shooterSystem = new ShooterSystem();
        xboxController = new XboxController(0);
        new JoystickButton(xboxController, XboxController.Button.kX.value).whileTrue(new ShootOut(shooterSystem));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        //new ShooterTeleop(shooterSystem,xboxController).schedule();
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
