package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder neoEncoder;
    private final DutyCycleEncoder dutyCycleEncoder;

    public ArmSystem() {
        motor = new CANSparkMax(RobotMap.ARM_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
        neoEncoder = motor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, RobotMap.ARM_NEO_ENCODER_CPR);
        dutyCycleEncoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER_PORT);

        motor.restoreFactoryDefaults();
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, RobotMap.ARM_MAX_ANGLE);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, RobotMap.ARM_MIN_ANGLE);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    }

    public void moveUp() {
        motor.set(0.5);
    }

    public void moveDown() {
        motor.set(-0.5);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getNeoEncoderVelocity() {
        return neoEncoder.getVelocity();
    }

    public double getNeoEncoderPosition() {
        return neoEncoder.getPosition() / RobotMap.ARM_GEAR_RATIO * 360;
    }

    public double getDutyCycleEncoderPosition() {
            // ↓ For offset the encoder to zero point that we want(floor),
            // ↓ we subtract from the encoder position the required quantity that need to offset the encoder to zero.
            // ↓ The "-" at the start we add to turn over the encoder value to positive value
        return -(dutyCycleEncoder.getAbsolutePosition() - RobotMap.ABSOLUTE_ENCODER_ZERO_OFFSET) * 360;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmNeoEncoderVelocity", getNeoEncoderVelocity());
        SmartDashboard.putNumber("ArmNeoEncoderPosition", getNeoEncoderPosition());
        SmartDashboard.putNumber("ArmDutyCycleEncoderPosition", getDutyCycleEncoderPosition());
    }
}