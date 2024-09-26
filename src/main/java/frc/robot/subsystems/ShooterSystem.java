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
    public final double SHOOTER_RPM_KP = 0.0001;
    public final double SHOOTER_RPM_KI = 0.000001;
    public final double SHOOTER_RPM_KD = 0;

    public ShooterSystem( ) {
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

        pid.setP(SHOOTER_RPM_KP,0);
        pid.setI(SHOOTER_RPM_KI, 0);
        pid.setD(SHOOTER_RPM_KD, 0);

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
        pid.setReference(targetRPM, CANSparkBase.ControlType.kVelocity);
    }

    public void setNewSoftLimits(int stallLimit, int freeLimit, int limitRPM){
        motorLT.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
        motorLB.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
        motorRT.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
        motorRB.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);

    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterLeftTopMotor", getLeftTopVelocityRpm());
        SmartDashboard.putNumber("ShooterLeftBottomMotor", getLeftBottomVelocityRpm());
        SmartDashboard.putNumber("ShooterRightTopMotor", getRightTopVelocityRpm());
        SmartDashboard.putNumber("ShooterRightBottomMotor", getRightBottomVelocityRpm());
        SmartDashboard.putNumber("ShooterLeftTopCurrent", motorLT.getOutputCurrent());
        SmartDashboard.putNumber("ShooterLeftBottomCurrent", motorLB.getOutputCurrent());
        SmartDashboard.putNumber("ShooterRightTopCurrent", motorRT.getOutputCurrent());
        SmartDashboard.putNumber("ShooterRightBottomCurrent", motorRB.getOutputCurrent());
    }
}
