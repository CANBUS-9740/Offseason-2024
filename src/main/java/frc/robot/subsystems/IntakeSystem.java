package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.util.Map;

public class IntakeSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final DigitalInput limitSwitch;
    private static final double DEFAULT_ROTATE_SPEED = 0.6;

    // Shuffleboard

    private GenericEntry noteInsideEntry;
    private GenericEntry motorSpeedEntry;

    public IntakeSystem() {
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH);

        motor.restoreFactoryDefaults();

        setUpShuffleboardTab();
    }

    private void setUpShuffleboardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake & Arm");

        noteInsideEntry = tab.add("Note Inside", false)
                .withPosition(5, 0)
                .withSize(2, 5)
                .getEntry();

        motorSpeedEntry = tab.add("Intake Motor Speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -5, "max", 5))
                .withPosition(0, 5)
                .withSize(7, 1)
                .getEntry();
    }

    public void out() {
        motor.set(-DEFAULT_ROTATE_SPEED);
    }

    public void in() {
        motor.set(DEFAULT_ROTATE_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isNoteInside() {
        return !limitSwitch.get();
    }

    public double getIntakeSpeed() {
        return motor.getEncoder().getVelocity() / 60 / RobotMap.INTAKE_MOTOR_TO_WHEEL_RATIO * RobotMap.INTAKE_WHEEL_CIRCUMFERENCE_METERS;
    }

    private void updateShuffleboard() {
        noteInsideEntry.setBoolean(isNoteInside());
        motorSpeedEntry.setDouble(getIntakeSpeed());
    }

    @Override
    public void periodic() {
        updateShuffleboard();

        SmartDashboard.putBoolean("IntakeIsNoteInside", isNoteInside());
    }
}
