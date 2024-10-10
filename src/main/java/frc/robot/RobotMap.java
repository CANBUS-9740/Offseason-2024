package frc.robot;

import edu.wpi.first.math.util.Units;

public class RobotMap {

    public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(3);
    public static final int DRIVE_LEFT_FRONT_MOTOR_ID = 4;
    public static final int DRIVE_LEFT_BACK_MOTOR_ID = 5;
    public static final int DRIVE_RIGHT_FRONT_MOTOR_ID = 2;
    public static final int DRIVE_RIGHT_BACK_MOTOR_ID = 3;
    public static final int PIGEON_ID = 7;
    public static final double TALON_ENCODER_PPR = 4096;

    public static final double DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO = 8.45;

    public final static int ARM_MOTOR_PORT = 8;
    public final static int ARM_ENCODER_PORT = 0;
    public final static double ARM_GEAR_RATIO = 100 / 1;
    public final static int ARM_NEO_ENCODER_CPR = 42;
    public final static double ARM_SHOOTER_ANGLE = 194;
    public final static double ARM_FLOOR_ANGLE = 12;
    public final static double ARM_MIN_ANGLE = 8;
    public final static double ARM_MAX_ANGLE = 195;
    public final static double ABSOLUTE_ENCODER_ZERO_OFFSET = 214 / 360.0;
    public final static double ARM_PID_P = 0.007;
    public final static double ARM_PID_I = 0.005;
    public final static double ARM_PID_D = 0.0;
    public final static int ARM_PID_I_ZONE = 10;
    public final static double ARM_PID_K_GRAVITY = 0.03;
    public final static int NEAR_ANGLE_TOLERANCE = 5;
    public final static double ARM_AMP_ANGLE = 138;
    public final static double ARM_AMP_RELEASE_ANGLE = 130;


    public static final int SHOOTER_MOTOR_LEFT_TOP = 13;
    public static final int SHOOTER_MOTOR_LEFT_BOTTOM = 12;
    public static final int SHOOTER_MOTOR_RIGHT_TOP = 15;
    public static final int SHOOTER_MOTOR_RIGHT_BOTTOM = 14;
    public static final int TARGET_RPM_SHOOTER = 5000;

    public static final int INTAKE_MOTOR = 9;
    public static final int INTAKE_LIMIT_SWITCH = 5;

    public static final int CLIMB_RIGHT_MOTOR = 00;
    public static final int CLIMB_LEFT_MOTOR = 00;
    public static final int CLIMB_LIMIT_SWITCH_RIGHT = 00;
    public static final int CLIMB_LIMIT_SWITCH_LEFT = 00;
    public static final int CLIMB_GEAR_RATIO = 0;
    public static final int CLIMB_RADIUS_CM = 3;
    public static final int CLIMB_ENCODER_PPR = 0;
    public static final double CLIMB_MOVE_POWER = 0.1;
    public static final double TARGET_STAY_IN_POSITION = 0;

}
