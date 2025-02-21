package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.util.PIDConstants; (Depreciated)
import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig; (PP removed replanning)
// import com.revrobotics.CANSparkMax; (Depreciated)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final int kDriverControllerPort2 = 1;
    public static final class Swerve {

        // Spark Max Idle Modes
        // public static final CANSparkMax.IdleMode driveIdleMode = CANSparkMax.IdleMode.kBrake; (Depreciated)
        // public static final CANSparkMax.IdleMode angleIdleMode = CANSparkMax.IdleMode.kBrake; (Depreciated)
        public static final IdleMode driveIdleMode = IdleMode.kBrake;
        public static final IdleMode angleIdleMode = IdleMode.kBrake;

        // Max Output Powers
        public static final double drivePower = 0.5;
        public static final double anglePower = 0.9;

        // Gyro
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        // Swerve Module Type
        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);
        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = chosenModule.canCoderInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final double angleGearRatio = chosenModule.angleGearRatio;
        // the number of degrees that a single rotation of the turn motor turns the wheel.
        public static final double DegreesPerTurnRotation = 360/angleGearRatio;
        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;

        // encoder setup
        // meters per rotation
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio );
        public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(19.25); // 19.25
        public static final double wheelBase = Units.inchesToMeters(19.05); // 19.05
        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        /* Angle Motor PID Values */
        public static final double angleKP = 0.05;
        public static final double angleKI = 0.0001;
        public static final double angleKD = 0.5;
        public static final double angleKFF = 0;

        /* Drive Motor info  */
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
                / driveGearRatio;

        /* Beam breaker info*/
        public static final int BeamBreakerPort = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.01;
        public static final double driveKI = 0.001;
        public static final double driveKD = 0.4;
        public static final double driveKFF = 1 / kDriveWheelFreeSpeedRps;
        /** Meters per Second */
        public static final double maxSpeed = 1; // 3.6576
        /** Radians per Second */
        public static final double maxAngularVelocity = 1; // 5.0
        public static double angleRampRate = 0;
        public static double speedRateLimit = 0.9;
        public static double kMaxSpeedMetersPerSecond = 7.2;

        // From PathPlanner
        // Error

        /* CanCoder Constants */
        public static final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

        public static class Modules {
            /* Module Specific Constants */

            /* Front Left Module - Module 1 */
            public static final class Mod0 {

                public static final int driveMotorID = 6;
                public static final int angleMotorID = 7;
                public static final int canCoderID = 33;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-68.291 + 180);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Front Right Module - Module 0 */
            public static final class Mod1 {
                public static final int driveMotorID = 5;
                public static final int angleMotorID = 4;
                public static final int canCoderID = 32;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-34.541);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Back Left Module - Module  2 */
            public static final class Mod2 {
                public static final int driveMotorID = 8;
                public static final int angleMotorID = 9;
                public static final int canCoderID = 31;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-150.556);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 10;
                public static final int angleMotorID = 11;
                public static final int canCoderID = 30;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-29.970);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
        }
    }

    public static final class CameraConstants {

        public static final double ROLL = -Math.PI / 2;
        public static final double PITCH = 0.0;
        public static final double YAW = 0.0;
        public static final Transform3d KCAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(8),
                        Units.inchesToMeters(22.125)), new Rotation3d(ROLL, PITCH, YAW)).inverse();

        public static final String CAMERA_NAME = "CSI";
        public static final double LARGEST_DISTANCE = 0.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double X_kP = 5;
        public static final double X_kI = 0;
        public static final double X_kD = 0;

        public static final double Y_kP = 5;
        public static final double Y_kI = 0;
        public static final double Y_kD = 0;

        public static final double THETA_kP = 6.2;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0;


        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort1 = 16; // 71=14?
    }

    public static final class ShooterConstants {
        public static final int kShooterMotorPort1 = 14; // 16
        public static final double SHOOTER_MAX_RPM = 5000;

        public static final int kAmpPort = 16; // 13

        public static final int beamBreakChannel = 0;
    }

    // SPARK MAXES NOT WORKING: 8, 13, 10, 12

    public static final class ElevatorConstants {
        public static final int kElevatorMotorPort1 = 12; // 12
        public static final int kElevatorMotorPort2 = 17; // 10

        // TODO: Test PID values
        public static final double elevatorkP = 0.5;
        public static final double elevatorkI = 0.00004;
        public static final double elevatorKD = 0.00004; 
    }

    public static final class PivotConstants {
        public static final int kPivotMotorPort1 = 50; 
        public static final int kPivotMotorPort2 = 51; // TODO: Update

        // TODO: Test PID values
        public static final int pivotkP = 1;
        public static final int pivotkI = 0;
        public static final int pivotKD = 0; 

    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}

