package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");    
    static NetworkTableEntry ta = table.getEntry("ta");
    static NetworkTableEntry tv = table.getEntry("tv");

    public static double x;
    public static double y;
    public static double area;
    public static boolean validTargets; 

    public static class LimelightData {

        /*---- Functions ----- */
        public static void update() {
            //read values periodically
            x = tx.getDouble(0.0);
            y = ty.getDouble(0.0);
            area = ta.getDouble(0.0);
            validTargets = tv.getBoolean(false);
            
        }

        public static double getX() {
            return x;
        }
    
        public static double getY() {
            return y;
        }
    
        public static double getArea() {
            return area;
        }

        public static boolean isValidTargets() {
            return validTargets;
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

