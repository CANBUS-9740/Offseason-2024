package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder NeoEncoder;
    private final DutyCycleEncoder dutyCycleEncoder;

    public ArmSystem() {
        motor = new CANSparkMax(RobotMap.ARM_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
        NeoEncoder = motor.getAlternateEncoder(RobotMap.ARM_ENCODER_CPR);
        dutyCycleEncoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER_PORT);

        dutyCycleEncoder.setPositionOffset(RobotMap.ARM_FLOOR_ANGLE / 360);
    }

    public void moveUp() {
        if (getDutyCycleEncoderPosition() >= RobotMap.ARM_MAX_ANGLE){
            stop();
        } else {
            motor.set(0.5);
        }
    }

    public void moveDown() {
        if (getDutyCycleEncoderPosition() <= RobotMap.ARM_MIN_ANGLE){
            stop();
        } else {
            motor.set(-0.5);
        }
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getNeoEncoderVelocity() {
        return NeoEncoder.getVelocity();
    }

    public double getNeoEncoderPosition() {
        return NeoEncoder.getPosition() / RobotMap.ARM_GEAR_RATIO * 360;
    }

    public double getDutyCycleEncoderPosition() {
        return dutyCycleEncoder.getAbsolutePosition() * 360;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmNeoEncoderVelocity", getNeoEncoderVelocity());
        SmartDashboard.putNumber("ArmNeoEncoderPosition", getNeoEncoderPosition());
        SmartDashboard.putNumber("ArmDutyCycleEncoderPosition", getDutyCycleEncoderPosition());
    }
}
