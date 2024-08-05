package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSystem extends SubsystemBase {
    private CANSparkMax motor;
    private DigitalInput limitSwitch;

    public IntakeSystem(){
        motor = new CANSparkMax(RobotMap.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH);
    }

    public void out(){
        motor.set(0.6);
    }

    public void in(){
        motor.set(-0.6);
    }

    public void stop(){
        motor.stopMotor();
    }

    public boolean isNoteInside(){
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isNoteInside", isNoteInside());
    }
}
