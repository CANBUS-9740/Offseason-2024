package frc.robot.utils;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class KeyPositions {
    private KeyPositions() {
        // Required private constructor
    }

    private static final Pose2d TOP_NOTE = new Pose2d(2.3, 7.0, new Rotation2d());
    private static final Pose2d MIDDLE_NOTE = new Pose2d(2.3, 5.55, new Rotation2d());
    private static final Pose2d BOTTOM_NOTE = new Pose2d(2.3,  4.04, new Rotation2d());

    private static final Pose2d FOURTH_CENTER_NOTE = new Pose2d(7.423, 2.414, new Rotation2d());

    private static final Pose2d TOP_THIRD = new Pose2d(6.0, 7.0, new Rotation2d());
    private static final Pose2d BOTTOM_THIRD = new Pose2d(6.0, 1.5, new Rotation2d());

    private static boolean shouldFlip() {
        return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    private static Pose2d getLocal(Pose2d pose) {
        return shouldFlip() ? GeometryUtil.flipFieldPose(pose) : pose;
    }

    public static Pose2d getLocalTopNote() {
        return getLocal(TOP_NOTE);
    }

    public static Pose2d getLocalMiddleNote() {
        return getLocal(MIDDLE_NOTE);
    }

    public static Pose2d getLocalBottomNote() {
        return getLocal(BOTTOM_NOTE);
    }

    public static Pose2d getFourthCenterNote() {
        return FOURTH_CENTER_NOTE;
    }

    public static Pose2d getLocalTopThird() {
        return getLocal(TOP_THIRD);
    }

    public static Pose2d getLocalBottomThird() {
        return getLocal(BOTTOM_THIRD);
    }
}
