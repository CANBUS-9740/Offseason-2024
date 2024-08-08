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
    public  final double SHOOTER_ROTATE_SPEED = 0.5;
    public final double SHOOTER_RPM_KP = 0;
    public final double SHOOTER_RPM_KI = 0;
    public final double SHOOTER_RPM_KD = 0;


    public ShooterSystem() {
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

    }
    public void stop() {
        motorLT.stopMotor();
        motorLB.stopMotor();
        motorRB.stopMotor();
        motorRT.stopMotor();
    }

    public void rotate() {
        motorLT.set(SHOOTER_ROTATE_SPEED);
        motorRB.set(SHOOTER_ROTATE_SPEED);
        motorLB.set(SHOOTER_ROTATE_SPEED);
        motorRT.set(SHOOTER_ROTATE_SPEED);
    }


    public double getLeftBottomVelocityRpm() {
        return encoderLB.getVelocity();
    }

    public double getLeftTopVelocityRpm() {
        return encoderLT.getVelocity();
    }
    public double getRightTopVelocityRpm() {
        return encoderRT.getVelocity();
    }
    public double getRightBottomVelocityRpm() {
        return encoderRB.getVelocity();
    }

    public void rotatePID(double targetRPM){
        pid.setReference(targetRPM, CANSparkBase.ControlType.kSmartMotion);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterLeftTopMotor", getLeftTopVelocityRpm());
        SmartDashboard.putNumber("ShooterLeftBottomMotor", getLeftBottomVelocityRpm());
        SmartDashboard.putNumber("ShooterRightTopMotor", getRightTopVelocityRpm());
        SmartDashboard.putNumber("ShooterRightBottomMotor", getRightBottomVelocityRpm());
    }
}
