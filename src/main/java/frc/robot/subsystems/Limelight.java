package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
 * Limelight 4's thermal output can be reduced by setting the
 * Throttle parameter to 100-200 while disabled, and to 0 while
 * enabled. Alternatively, you can switch to a viewfinder pipeline 
 * while disabled.
 */

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

        public static void setIdle() {
            // Set the camera to process only every 150 frames
            // Set to 100-200 while disabled
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(150);
        }

        public static void setOn() {
            // Set the camera to process every frame
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);
        }
    }
}