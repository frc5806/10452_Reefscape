package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.Swerve.AlignLimelight;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;

public class Limelight {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");    
    static NetworkTableEntry ta = table.getEntry("ta");
    static NetworkTableEntry tv = table.getEntry("tv");
    static NetworkTableEntry targetpose_robotspace = table.getEntry("targetpose_robotspace");

    public static double x;
    public static double y; 
    public static double area;
    public static boolean validTargets; 
    public static double[] targetpose;

    public static class LimelightData {

        /*---- Functions ----- */
        public static void update() {
            //read values periodically from the network tables, which are updated 100 times a second from the limelight

            //This gets the x offset
            x = tx.getDouble(0.0);

            //THis gets the y offset
            y = ty.getDouble(0.0);

            //This gets the area of the april tag relative to the camera - so 0 means it can't be seen and 100 means it occupies the entire screen
            area = ta.getDouble(0.0);

            //This gets if there is a valid target which is also stored in the network tables.
            validTargets = tv.getBoolean(false);

            // Get the actual targeting data
            targetpose = targetpose_robotspace.getDoubleArray(new double[6]);

        }

        //A series of getters

        public static double getX() {
            return x;
        }
    
        public static double getY() {
            return y;
        }
    
        public static double getArea() {
            return area;
        }

        public static Boolean isValidTargets() {
            return validTargets;
        }

        public static double[] getTargetpose() {
            return targetpose;
        }

        // Prints out on terminal x, y, z values from tables
        public static void dataTest() {

            LimelightData.update();
            double[] dataXYA = new double[]{LimelightData.getX(), LimelightData.getY(), LimelightData.getArea()};

            System.out.println("X: " + String.valueOf(dataXYA[0]) + ", Y: " + String.valueOf(dataXYA[1]) + ", Area: " + String.valueOf(dataXYA[2]));
            System.out.println("ValidTargets: " + Boolean.valueOf(validTargets));
        }

    }
}