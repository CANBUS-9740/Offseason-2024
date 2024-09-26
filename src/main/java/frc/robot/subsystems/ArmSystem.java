package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import static frc.robot.RobotMap.NEAR_ANGLE_TOLERANCE;

public class ArmSystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder neoEncoder;
    private final DutyCycleEncoder absEncoder;

    // Shuffleboard

    private GenericEntry angleEntry;

    public ArmSystem() {
        motor = new CANSparkMax(RobotMap.ARM_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
        neoEncoder = motor.getEncoder();
        absEncoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER_PORT);

        motor.restoreFactoryDefaults();

        setUpShuffleboardTab();
    }

    private void setUpShuffleboardTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake & Arm");

        angleEntry = tab.add("Arm Angle", 0.0)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(0, 0)
                .withSize(5, 5)
                .getEntry();
    }

    public boolean reachedATargetAngle(double targetAngle) {
        return MathUtil.isNear(targetAngle, getAbsEncoderPositionDegrees(), NEAR_ANGLE_TOLERANCE);
    }

    public void move(double power) {
        if ((power < 0 && getAbsEncoderPositionDegrees() > RobotMap.ARM_MAX_ANGLE) || power > 0 && getAbsEncoderPositionDegrees() < RobotMap.ARM_MIN_ANGLE){
            stop();
        } else {
            motor.set(power);
        }
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getNeoEncoderVelocityRPM() {
        return neoEncoder.getVelocity();
    }

    public double getNeoEncoderPositionDegrees() {
        return (-neoEncoder.getPosition() / RobotMap.ARM_GEAR_RATIO) * 360;
    }

    public double getAbsEncoderPositionDegrees() {
            // ↓ For offset the encoder to zero point that we want(floor),
            // ↓ we subtract from the encoder position the required quantity that need to offset the encoder to zero.
            // ↓ The "-" at the start we add to turn over the encoder value to positive value
        return -(absEncoder.getAbsolutePosition() - RobotMap.ABSOLUTE_ENCODER_ZERO_OFFSET) * 360;
    }

    private void updateShuffleboard() {
        angleEntry.setDouble(getAbsEncoderPositionDegrees());
    }

    @Override
    public void periodic() {
        updateShuffleboard();

        SmartDashboard.putNumber("ArmNeoEncoderVelocity", getNeoEncoderVelocityRPM());
        SmartDashboard.putNumber("ArmNeoEncoderPosition", getNeoEncoderPositionDegrees());
        SmartDashboard.putNumber("ArmDutyCycleEncoderPosition", getAbsEncoderPositionDegrees());
    }
}