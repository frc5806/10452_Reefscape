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
    static NetworkTableEntry tv_reef = table.getEntry("tv_reef");
    static NetworkTableEntry targetpose_robotspace_reef = table.getEntry("targetpose_robotspace_reef");
    static NetworkTableEntry tv_coral = table.getEntry("tv_coral");
    static NetworkTableEntry targetpose_robotspace_coral = table.getEntry("targetpose_robotspace_coral");

    public static long isTargetReef; 
    public static long isTargetCoral; 

    public static double[] targetposeReef;
    public static double[] targetposeCoral;


    public static class LimelightData {
        public static void update() {
            // Boolean if an AprilTag is detected
            isTargetReef = tv_reef.getInteger(0);
            isTargetCoral = tv_coral.getInteger(0);

            // AprilTag location in Robot coordinate system {tx, ty, tz, roll, pitch, yaw}
            targetposeReef = targetpose_robotspace_reef.getDoubleArray(new double[6]);
            targetposeCoral = targetpose_robotspace_coral.getDoubleArray(new double[6]);
        }

        public static Boolean isValidTarget() {
            update();
            return (isTargetReef == 1 || isTargetCoral == 1);
        }

        public static double[] getTargetposeReef() {
            update();
            return targetposeReef;
        }

        public static double[] getTargetposeCoral() {
            update();
            return targetposeCoral;
        }

        public static void setIdle() {
            // Set the camera to process only every 150 frames
            // Set to 100-200 while disabled
            table.getEntry("throttle_set").setNumber(0);
        }

        public static void setOn() {
            // Set the camera to process every frame
            table.getEntry("throttle_set").setNumber(0);
        }
    }
}