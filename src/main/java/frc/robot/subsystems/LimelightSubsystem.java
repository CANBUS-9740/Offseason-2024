package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.Arrays;

public class LimelightSubsystem extends SubsystemBase {
    private NetworkTable table;
    private double[] botPoseArray;
    private NetworkTableEntry networkTableEntry;
    private Field2d field2d;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        networkTableEntry = table.getEntry("botpose_wpiblue");
        field2d = new Field2d();

        SmartDashboard.putData("LimelightField", field2d);
        botPoseArray = networkTableEntry.getDoubleArray(new double[6]);

        for (double num : botPoseArray) {
            System.out.println(num);
        }
    }

    public double getX() {
        return botPoseArray[0];
    }

    public double getY() {
        return botPoseArray[1];
    }

    public double getZ() {
        return botPoseArray[2];
    }

    public double getRoll() {
        return botPoseArray[3];
    }

    public double getPitch() {
        return botPoseArray[4];
    }

    public double getYaw() {
        return botPoseArray[5];
    }

    @Override
    public void periodic() {
        double[] array = new double[6];
        Arrays.fill(array, -10.0);

        botPoseArray = networkTableEntry.getDoubleArray(array);
        SmartDashboard.putNumber("yaw: ", getYaw());
        SmartDashboard.putNumber("Pitch: ", getPitch());
        SmartDashboard.putNumber("Roll: ", getRoll());
        SmartDashboard.putNumber("Z: ", getZ());
        SmartDashboard.putNumber("Y: ", getY());
        SmartDashboard.putNumber("X: ", getX());
    }
}