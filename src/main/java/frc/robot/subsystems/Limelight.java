package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tv = table.getEntry("tv");
    static NetworkTableEntry targetpose_robotspace = table.getEntry("targetpose_robotspace");

    public static boolean isTarget; 
    public static double[] targetpose;

    public static class LimelightData {
        public static void update() {
            // Boolean if an AprilTag is detected
            isTarget = tv.getBoolean(false);

            // AprilTag location in Robot coordinate system {tx, ty, tz, roll, pitch, yaw}
            targetpose = targetpose_robotspace.getDoubleArray(new double[6]);
        }

        public static Boolean isValidTarget() {
            update();
            return isTarget;
        }

        public static double[] getTargetpose() {
            update();
            return targetpose;
        }
    }
}