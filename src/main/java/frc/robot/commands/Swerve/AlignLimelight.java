package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.LimelightValues;
import frc.robot.subsystems.Limelight.LimelightData;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.Limelight;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class AlignLimelight extends Command {
    private SwerveBase Swerve;
    private double lateral_offset = LimelightValues.lateral_offset;
    private double longitudinal_offset = LimelightValues.longitudinal_offset;

    public AlignLimelight(
            SwerveBase Swerve,
            double lateral_offset,
            double longitudinal_offset) {
        this.Swerve = Swerve;
        this.lateral_offset = lateral_offset;
        this.longitudinal_offset = longitudinal_offset;
        addRequirements(Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double strafeVal = Swerve.strafeLimelight(lateral_offset) * 2;
        // Could add strafe value as well (using x value rather than turning)
        double inOutVal = Swerve.rangeLimelight(longitudinal_offset) * 1.5;
        double rotationVal = 0.03 * Swerve.aimLimelight() * Math.abs(1 / (10*Math.pow(strafeVal, 2) + 1));
        // double rotationVal = 0.03 * Swerve.aimLimelight();

        //Updates numbers to smart dashbaord
        SmartDashboard.putNumber("Limelight inOutVal", inOutVal);
        SmartDashboard.putNumber("strafeVal", strafeVal);
        SmartDashboard.putNumber("Limelight rotationVal", rotationVal);

        /* Drive */
        
        //Drives the robot after getting translation values above

        //We could in addition get the type of april tag and do different things depending on the type of april tag.
        Swerve.drive(
                new Translation2d(inOutVal, strafeVal).times(Constants.Swerve.maxSpeed).times(0.3),
                // new Translation2d(0, 0),
                rotationVal * Constants.Swerve.maxAngularVelocity * (0.4),
                // 0,
                false, 
                true
        );

        //Maybe we should run this command again for a second time to get some fine tuned adjustments and get closer to the location, or use the datatest() method in Limelight
    }

    @Override
    public boolean isFinished() {

        //Tests to make sure limelight is within accetable bounds of the april tag

        double strafeVal = Swerve.strafeLimelight(lateral_offset);
        // Could add strafe value as well (using x value rather than turning)
        double inOutVal = Swerve.rangeLimelight(longitudinal_offset);
        double rotationVal = Swerve.aimLimelight();

        boolean rotation_in_bounds = Math.abs(rotationVal) < LimelightValues.maxRotationError ? true : false;
        boolean strafe_in_bounds = Math.abs(strafeVal) < LimelightValues.maxSideToSideError ? true : false;
        System.out.println(strafeVal);
        System.out.println(inOutVal);
        boolean in_out_in_bounds = Math.abs(inOutVal) < LimelightValues.maxInOutError ? true : false;
        System.out.println(rotation_in_bounds + " " + strafe_in_bounds + " " + in_out_in_bounds);
        return (rotation_in_bounds && strafe_in_bounds && in_out_in_bounds);
    }
 
    @Override
    public void end(boolean interrupted){

        //When the command ends stop the robot moving
        Swerve.drive(
            new Translation2d(0, 0),
            0,
            false, 
            true
        );
    }
}
