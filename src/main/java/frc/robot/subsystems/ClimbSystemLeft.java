package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSystemLeft extends SubsystemBase {
    private final CANSparkMax motorLeft;

    private final DigitalInput limitSwitchLeft;

    private final RelativeEncoder encoderLeft;

    private SparkPIDController pid;

    public ClimbSystemLeft() {
        motorLeft = new CANSparkMax(RobotMap.CLIMB_LEFT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        encoderLeft = motorLeft.getEncoder();
        limitSwitchLeft = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_LEFT);

        motorLeft.setSoftLimit(SoftLimitDirection.kForward, fromHeightToRotations(90));
        motorLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);

        pid.setP(0,0);
        pid.setI(0,0);
        pid.setD(0,0);

    }


    public void stop() {
        motorLeft.stopMotor();
    }

    public void stayInPositionLeft(double targetLeft){
        pid.setReference(targetLeft, CANSparkBase.ControlType.kVelocity);
    }

    public void move(double power) {
        motorLeft.set(power);
    }

    public boolean isLeftClimberDown() {
        return limitSwitchLeft.get();
    }

    public void resetLeft() {
        encoderLeft.setPosition(0);
    }


    public double getHeight() {
        return (encoderLeft.getPosition() / RobotMap.CLIMB_GEAR_RATIO * (RobotMap.CLIMB_RADIUS_CM * 2 * Math.PI));
    }

    public float fromHeightToRotations(double height) {
        return (float) (height * RobotMap.CLIMB_GEAR_RATIO / (RobotMap.CLIMB_RADIUS_CM * 2 * Math.PI));
    }

    public void periodic() {
        SmartDashboard.putNumber("ClimbHeightLeft", getHeight());
    }
}
