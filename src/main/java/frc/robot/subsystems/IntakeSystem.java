package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final DigitalInput limitSwitch;
    private static final double DEFAULT_ROTATE_SPEED = 0.6;
    private static final double SLOW_IN_ROTATE_SPEED = 0.1;

    public IntakeSystem() {
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(20,5);
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

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("IntakeIsNoteInside", isNoteInside());
        SmartDashboard.putNumber("IntakeCurrent", motor.getOutputCurrent());

    }

}
