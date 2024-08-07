package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final CANSparkMax motorLT;
    private final CANSparkMax motorRT;
    private final CANSparkMax motorLB;
    private final CANSparkMax motorRB;
    private final RelativeEncoder encoderLT;
    private final RelativeEncoder encoderRT;
    private final RelativeEncoder encoderLB;
    private final RelativeEncoder encoderRB;
    private final SparkPIDController pid;
    private final double targetRPM;

    public ShooterSystem(double targetRPM) {
        motorLT = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_TOP, CANSparkLowLevel.MotorType.kBrushless);
        motorRT = new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_TOP, CANSparkLowLevel.MotorType.kBrushless);
        motorLB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
        motorRB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);

        motorRT.restoreFactoryDefaults();
        motorLT.restoreFactoryDefaults();
        motorLB.restoreFactoryDefaults();
        motorRB.restoreFactoryDefaults();

        motorRT.setInverted(true);
        motorRB.setInverted(true);

        encoderLT = motorLT.getEncoder();
        encoderLB = motorLB.getEncoder();
        encoderRT = motorRT.getEncoder();
        encoderRB = motorRB.getEncoder();

        pid = motorLB.getPIDController();

        pid.setP(0.002,0);
        pid.setI(0, 0);
        pid.setD(0.042, 0);

        motorRB.follow(motorLB);
        motorRT.follow(motorLB);
        motorLT.follow(motorLB);


        this.targetRPM = targetRPM;
    }
    public void stop() {
        motorLT.stopMotor();
        motorLB.stopMotor();
        motorRB.stopMotor();
        motorRT.stopMotor();
    }

    public void rotate() {
        motorLT.set(RobotMap.SHOOTER_ROTATE_SPEED);
        motorRB.set(RobotMap.SHOOTER_ROTATE_SPEED);
        motorLB.set(RobotMap.SHOOTER_ROTATE_SPEED);
        motorRT.set(RobotMap.SHOOTER_ROTATE_SPEED);
    }


    public double getVelocityLB() {
        return encoderLB.getVelocity();
    }

    public double getVelocityLT() {
        return encoderLT.getVelocity();
    }
    public double getVelocityRT() {
        return encoderRT.getVelocity();
    }
    public double getVelocityRB() {
        return encoderRB.getVelocity();
    }

    public void rotatePID(){
        pid.setReference(targetRPM, CANSparkBase.ControlType.kSmartMotion);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterLeftTopMotor", getVelocityLT());
        SmartDashboard.putNumber("ShooterLeftBottomMotor", getVelocityLB());
        SmartDashboard.putNumber("ShooterRightTopMotor", getVelocityRT());
        SmartDashboard.putNumber("ShooterRightBottomMotor", getVelocityRB());
    }
}
