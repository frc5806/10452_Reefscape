package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.lib.math.GeometryUtils;
import frc.robot.Constants;
import frc.robot.util.NavXGyro;

public class SwerveBase extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometer;
    public RevSwerveModule[] swerveMods;
    public NavXGyro gyro = NavXGyro.getInstance();

    private int moduleSynchronizationCounter = 0;
    private double avgOmega = 0;           
    private RobotConfig config;


    //private Rotation2d fieldOffset = new Rotation2d(gyro.getYaw()).rotateBy(new Rotation2d(180));
    private final Field2d field = new Field2d();
    private boolean hasInitialized = false;

    public SwerveBase() {

        //Establishes 4 different Swerve modules
        swerveMods = new RevSwerveModule[] {

            new RevSwerveModule(0, Constants.Swerve.Modules.Mod0.constants),
            new RevSwerveModule(1, Constants.Swerve.Modules.Mod1.constants),
            new RevSwerveModule(2, Constants.Swerve.Modules.Mod2.constants),
            new RevSwerveModule(3, Constants.Swerve.Modules.Mod3.constants)
        };

        swerveOdometer = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        resetOdometry(new Pose2d());
        
        //um?
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        
        // PathPlanner
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0, 0), // Translation PID constants
                    new PIDConstants(5, 0, 0) // Rotation PID constants
            ),
            config,
            () -> {
               // Boolean supplier that controls when the path will be mirrored for the red alliance
               // This will flip the path being followed to the red side of the field.
               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }

    //I don't think this is implemented anywhere
    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
                false);
    }

    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        //This method I believe takes in the initial changes that the drive command takes in, then adjusts them with th Twist2D.

        //I think the values are mulitplied and divided by LOOP_TIME_S because Twist2D needs to take into account the "time interval" of each movemenet of the movement, though are divded because chasisspeeds does not need that - I think

        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));

        //Twist2D is the difference between two positions - between two different pose2Ds (curves)
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);

        //Creates new speeds based on the Twist2D just created. dx, dy, and dtheta are created by the Twist2D.
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    //Twist2D represents a difference between two positions, ChassisSpeeds is the speed of the chasis (I think its a vector vs magnitude thing)

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                swerveOdometer.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(DriverStation.getAlliance().equals(DriverStation.Alliance.Red) ? 180 : 0))
        )
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
        );

        //Update Chasis speeds
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        //Give each swerve module the new chasis speeds
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        
        //When we add the new chasis speeds to each swerve module sometimes some of the swerve modules have a speed greater than the maximum speed, even if the initial speed and the added speed are below the max speed, so this method jsut normalizes the speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }

    private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        }
        speeds = ChassisSpeeds.discretize(speeds, avgOmega);
        var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);
    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }
    public Pose2d getPose() {
        return swerveOdometer.getEstimatedPosition();
    }
    public void resetOdometry(Pose2d pose) {
        swerveOdometer.resetPosition(new Rotation2d(), getModulePositions(), pose);
         zeroGyro();
        //zeroGyro(pose.getRotation().getDegrees())
    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360).minus(gyro.getRotation2d()) : gyro.getRotation2d();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public double getPitch() {
        return gyro.getRoll();
    }

    public void synchronizeModuleEncoders() {
        for(RevSwerveModule mod : swerveMods) {
            mod.synchronizeEncoders();
        }
    }

    public double getAvgOmega() {
        double sum = 0;
        for(RevSwerveModule mod : swerveMods) {
            sum += Math.abs(mod.getOmega());
        }
        return sum / 4;
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        this.drive(new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond),speeds.omegaRadiansPerSecond,false,false);
    //  this.drive(speeds, false);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.Swerve.swerveKinematics.toChassisSpeeds(swerveMods[0].getState(),
                                                                swerveMods[1].getState(),
                                                                swerveMods[2].getState(),
                                                                swerveMods[3].getState()), getYaw());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro", gyro.getYaw());

        swerveOdometer.update(getYaw(), getModulePositions());
        // SmartDashboard.putBoolean("photonGood", cam.latency() < 0.6);
        if (!hasInitialized /* || DriverStation.isDisabled() */) {
        //     var robotPose = new Pose2d(new Translation2d(0,0), getYaw());//cam.getInitialPose();
        //     if (robotPose.isPresent()) {
        //         swerveOdometer.resetPosition(getYaw(), getModulePositions(), robotPose.get());
                hasInitialized = true;
        //     }
        //  } else {
        //     var result = cam.getEstimatedGlobalPose(swerveOdometer.getEstimatedPosition());
        //     if (result.isPresent()) {
        //         var camPose = result.get();
        //         if (camPose.targetsUsed.get(0).getArea() > 0.7) {
        //             swerveOdometer.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
        //                     camPose.timestampSeconds);
        //         }
        //         field.getObject("Cam Est Pose").setPose(camPose.estimatedPose.toPose2d());
        //     } else {
            //    field.getObject("Cam Est Pose").setPose(new Pose2d(0, 0, new Rotation2d(0,0)));
            // }
        }

        SmartDashboard.putData("field", field);

        field.setRobotPose(getPose());
        //aprilTagTarget.setBoolean(cam.seesTarget());

        avgOmega = getAvgOmega();

        for(SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
        // lib)
        // To ensure that everytime we initialize it works.
        if (avgOmega <= .03 && ++moduleSynchronizationCounter > 20)
        {
            SmartDashboard.putBoolean("Synchronizing Encoders", !SmartDashboard.getBoolean("Synchronizing Encoders", false));
            synchronizeModuleEncoders();
            moduleSynchronizationCounter = 0;
        }
        if(avgOmega <= .005){
            SmartDashboard.putBoolean("Can Synchronizing Encoders", true);
        }else {
            SmartDashboard.putBoolean("Can Synchronizing Encoders", false);
        }
        SmartDashboard.putNumber("avgOmega", avgOmega);

        SmartDashboard.putBoolean("isRed", DriverStation.getAlliance().equals(DriverStation.Alliance.Red));
    }

    public void stop() {
        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(mod.getState(), false);
        }
    }
}
