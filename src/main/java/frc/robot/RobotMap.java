package frc.robot;

public class RobotMap {

    //                          ARM
    // ▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
    public final static int ARM_MOTOR_PORT = 8;
    public final static int ARM_ENCODER_PORT = 0;
    public final static double ARM_GEAR_RATIO = 100 / 1;
    public final static int ARM_NEO_ENCODER_CPR = 42;
    public final static double ARM_SHOOTER_ANGLE = 190;
    public final static double ARM_FLOOR_ANGLE = 15;
    public final static double ARM_MIN_ANGLE = 8;
    public final static double ARM_MAX_ANGLE = 195;
    public final static double ABSOLUTE_ENCODER_ZERO_OFFSET = 214 / 360.0;
    public final static double ARM_PID_P = 0.01;
    public final static double ARM_PID_I = 0.005;
    public final static double ARM_PID_D = 0.0;

    public static final int SHOOTER_MOTOR_LEFT_TOP = 13;
    public static final int SHOOTER_MOTOR_LEFT_BOTTOM = 12;
    public static final int SHOOTER_MOTOR_RIGHT_TOP = 15;
    public static final int SHOOTER_MOTOR_RIGHT_BOTTOM = 14;

    public static final int INTAKE_MOTOR = 9;
    public static final int INTAKE_LIMIT_SWITCH = 5;

}
