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
    static NetworkTable table_reef = NetworkTableInstance.getDefault().getTable("limelight-reef");
    static NetworkTableEntry tv_reef = table_reef.getEntry("tv");
    static NetworkTableEntry targetpose_robotspace_reef = table_reef.getEntry("targetpose_robotspace");

    public static long isTargetReef; 

    public static double[] targetposeReef;


    public static class LimelightData {
        public static void update() {
            // Boolean if an AprilTag is detected
            isTargetReef = tv_reef.getInteger(0);

            // AprilTag location in Robot coordinate system {tx, ty, tz, roll, pitch, yaw}
            targetposeReef = targetpose_robotspace_reef.getDoubleArray(new double[6]);
        }

        public static Boolean isValidTarget() {
            update();
            return (isTargetReef == 1);
        }

        public static double[] getTargetposeReef() {
            update();
            return targetposeReef;
        }

        public static void setIdle() {
            // Set the camera to process only every 150 frames
            // Set to 100-200 while disabled
            table_reef.getEntry("throttle_set").setNumber(0);
        }

        public static void setOn() {
            // Set the camera to process every frame
            table_reef.getEntry("throttle_set").setNumber(0);
        }
    }
}