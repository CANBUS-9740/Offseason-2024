package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder neoEncoder;
    private final DutyCycleEncoder absEncoder;

    public ArmSystem() {
        motor = new CANSparkMax(RobotMap.ARM_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
        neoEncoder = motor.getEncoder();
        absEncoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER_PORT);

        motor.restoreFactoryDefaults();

        neoEncoder.setPosition((absEncoder.getAbsolutePosition() - RobotMap.ABSOLUTE_ENCODER_ZERO_OFFSET) / RobotMap.ARM_GEAR_RATIO);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) ((RobotMap.ARM_MAX_ANGLE * RobotMap.ARM_GEAR_RATIO) / 360));
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) ((RobotMap.ARM_MIN_ANGLE * RobotMap.ARM_GEAR_RATIO) / 360));
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

    public double getNeoEncoderVelocityRPM() {
        return neoEncoder.getVelocity();
    }

    public double getNeoEncoderPositionDegrees() {
        return neoEncoder.getPosition() / RobotMap.ARM_GEAR_RATIO * 360;
    }

    public double getAbsEncoderPositionDegrees() {
            // ↓ For offset the encoder to zero point that we want(floor),
            // ↓ we subtract from the encoder position the required quantity that need to offset the encoder to zero.
            // ↓ The "-" at the start we add to turn over the encoder value to positive value
        return -(absEncoder.getAbsolutePosition() - RobotMap.ABSOLUTE_ENCODER_ZERO_OFFSET) * 360;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmNeoEncoderVelocity", getNeoEncoderVelocityRPM());
        SmartDashboard.putNumber("ArmNeoEncoderPosition", getNeoEncoderPositionDegrees());
        SmartDashboard.putNumber("ArmDutyCycleEncoderPosition", getAbsEncoderPositionDegrees());
    }
}