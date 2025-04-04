package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.LimelightValues;
import frc.robot.subsystems.Limelight.LimelightData;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.commands.Swerve.TimedDrive;


public class AlignLimelightReef extends Command {
    private SwerveBase Swerve;
    private double lateral_offset = 0;
    private double longitudinal_offset = 0;

    public AlignLimelightReef(
            SwerveBase Swerve,
            double lateral_offset,
            double longitudinal_offset) {
        this.Swerve = Swerve;
        this.lateral_offset = lateral_offset;
        this.longitudinal_offset = longitudinal_offset;
        addRequirements(Swerve);
    }

    public double[] AimLimelight(double lateral_offset, double longitudinal_offset) {
        double[] targetpose = LimelightData.getTargetposeReef(); // AprilTag location in Robot coordinate system {tx, ty, tz, roll, pitch, yaw}

        double tx = targetpose[0];
        double tz = targetpose[2];
        double y_rot = targetpose[4];

        double translationSpeed = (tz - longitudinal_offset);
        double strafeSpeed = -(tx - lateral_offset);
        double angularVelocity = -y_rot;

        return new double[] {translationSpeed, strafeSpeed, angularVelocity};
    }

    private WaitCommand walkForward() {
        return new TimedDrive(Swerve, 1.0, 0, 0.5, 0);
    }

    @Override
    public void initialize() {
        LimelightData.setOn();
    }

    @Override
    public void execute() {
        double[] corrections = AimLimelight(lateral_offset, longitudinal_offset);

        double longitudinalVal = corrections[0] * 1.5;
        double lateralVal = corrections[1] * 2;
        double rotationVal = 0.03 * corrections[2] * Math.abs(1 / (10*Math.pow(lateralVal, 2) + 1));


        //Updates numbers to smart dashbaord
        SmartDashboard.putNumber("Limelight longitudinalVal", longitudinalVal);
        SmartDashboard.putNumber("Limelight lateralVal", lateralVal);
        SmartDashboard.putNumber("Limelight rotationVal", rotationVal);

        /* Drive */
        Swerve.drive(
                new Translation2d(longitudinalVal, lateralVal).times(Constants.Swerve.maxSpeed).times(0.3),
                rotationVal * Constants.Swerve.maxAngularVelocity * (0.4),
                false, 
                true
        );

        //Maybe we should run this command again for a second time to get some fine tuned adjustments and get closer to the location, or use the datatest() method in Limelight
    }

    @Override
    public boolean isFinished() {
        //Tests to make sure limelight is within accetable bounds of the april tag
        double[] corrections = AimLimelight(lateral_offset, longitudinal_offset);

        boolean in_out_in_bounds = Math.abs(corrections[0]) < LimelightValues.maxInOutError ? true : false;
        boolean strafe_in_bounds = Math.abs(corrections[1]) < LimelightValues.maxSideToSideError ? true : false;
        boolean rotation_in_bounds = Math.abs(corrections[2]) < LimelightValues.maxRotationError ? true : false;
    
        return (rotation_in_bounds && strafe_in_bounds && in_out_in_bounds);
    }
 
    @Override
    public void end(boolean interrupted){
        walkForward();

        //When the command ends stop the robot moving
        Swerve.drive(
            new Translation2d(0, 0),
            0,
            false, 
            true
        );

        LimelightData.setIdle();
    }
}
