package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSystemRight extends SubsystemBase {
    private final CANSparkMax motorRight;

    private final DigitalInput limitSwitchRight;

    private final RelativeEncoder encoderRight;

    private SparkPIDController pid;

    public ClimbSystemRight() {
        motorRight = new CANSparkMax(RobotMap.CLIMB_RIGHT_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        encoderRight = motorRight.getEncoder();
        limitSwitchRight = new DigitalInput(RobotMap.CLIMB_LIMIT_SWITCH_RIGHT);

        motorRight.setSoftLimit(SoftLimitDirection.kForward, fromHeightToRotations(90));
        motorRight.setSoftLimit(SoftLimitDirection.kReverse, 0);

        pid.setP(0,0);
        pid.setI(0,0);
        pid.setD(0,0);

    }

    public void stop() {
        motorRight.stopMotor();
    }

    public void stayInPositionRight(double targetRight){
         pid.setReference(targetRight, CANSparkBase.ControlType.kVelocity);
    }


    public void move(double power) {
        motorRight.set(power);
    }


    public boolean isRightClimberDown() {
        return limitSwitchRight.get();
    }


    public void resetRight() {
        encoderRight.setPosition(0);
    }

    public double getHeight() {
        return (encoderRight.getPosition() / RobotMap.CLIMB_GEAR_RATIO * (RobotMap.CLIMB_RADIUS_CM * 2 * Math.PI));
    }


    public float fromHeightToRotations(double height) {
        return (float) (height * RobotMap.CLIMB_GEAR_RATIO / (RobotMap.CLIMB_RADIUS_CM * 2 * Math.PI));
    }

    public void periodic() {
        SmartDashboard.putNumber("ClimbHeightRight", getHeight());
    }
}
