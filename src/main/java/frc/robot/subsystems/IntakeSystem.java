package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardDashboard;
import frc.robot.utils.ShuffleboardUtils;

public class IntakeSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final DigitalInput limitSwitch;
    private static final double DEFAULT_ROTATE_SPEED = 0.6;
    private static final double SLOW_IN_ROTATE_SPEED = 0.1;

    // Shuffleboard

    private GenericEntry motorSpeedEntry;
    private GenericEntry noteInsideEntry;

    public IntakeSystem() {
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(20,5);

        setUpShuffleboard();

        ShuffleboardDashboard.setIntakeDataSupplier(() -> new ShuffleboardDashboard.IntakeData(
                isNoteInside(),
                getIntakeSpeedRpm()
        ));
    }

    public void out() {
        motor.set(-DEFAULT_ROTATE_SPEED);
    }

    public void in() {
        motor.set(DEFAULT_ROTATE_SPEED);
    }

    public void slowIn() {
        motor.set(SLOW_IN_ROTATE_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean isNoteInside() {
        return !limitSwitch.get();
    }

    public double getIntakeSpeedRpm() {
        return motor.getEncoder().getVelocity() / RobotMap.INTAKE_MOTOR_TO_WHEEL_RATIO;
    }

    private void setUpShuffleboard() {
        ShuffleboardTab tab = ShuffleboardUtils.getArmIntakeShooterTab();

        motorSpeedEntry = ShuffleboardUtils.addIntakeMotorSpeedWidget(tab)
                .withPosition(0, 0)
                .withSize(7, 1)
                .getEntry();

        noteInsideEntry = tab.add("Note Inside", false)
                .withPosition(0, 5)
                .withSize(13, 1)
                .getEntry();

        ShuffleboardLayout subsystemsLayout = ShuffleboardUtils.getArmIntakeShooterSubsystemsLayout();
        subsystemsLayout.add("Intake", this)
                .withPosition(1, 0);
    }

    private void updateShuffleboard() {
        noteInsideEntry.setBoolean(isNoteInside());
        motorSpeedEntry.setDouble(getIntakeSpeedRpm());
    }

    @Override
    public void periodic() {
        updateShuffleboard();
    }

}
