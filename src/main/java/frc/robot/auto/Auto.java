package frc.robot.auto;

// import com.pathplanner.lib.commands.FollowPathHolonomic; (Depreciated)
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.SwerveBase;

public class Auto extends Command {
    SwerveBase swerve;
    Intake intake;

    public Auto(SwerveBase swerve, Intake intake) {
        this.swerve = swerve;
        this.intake = intake;
        addRequirements(swerve, intake);
    }


    // public Command driveToNote() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("Blue"); // PickUpRing

    //     // return new FollowPathHolonomic( (Depreciated)
    //     return new FollowPathCommand(
    //             path,
    //             swerve::getPose, // Robot pose supplier
    //             swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //             Constants.Swerve.pathFollowerConfig,
    //             () -> {
    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                     return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //             },
    //             swerve // Reference to this subsystem to set requirements
    //     );
    // }

    // Error
    // public Command driveToShooter() {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");

    //     // return new FollowPathHolonomic( (Depreciated)
    //     return new FollowPathCommand(
    //             path,
    //             swerve::getPose, // Robot pose supplier
    //             swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //             Constants.Swerve.robotConfig,
    //             () -> {
    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                     return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //             },
    //             swerve // Reference to this subsystem to set requirements
    //     );
    // }

    public Command RingAuto(){
        String autoName = "RingAuto";
        return new PathPlannerAuto(autoName);
    }

    // public Command Intake(){
    //     return Commands.sequence(intake.runIntake(0.9).withTimeout(3));
    // }

    // public Command Path() {
    //     return driveToNote();
    // }

    public Command TwoRingAuto() {
        return new PathPlannerAuto("2RingAuto");
        // .andThen(Commands.sequence(intake.runIntake(0.9).withTimeout(3)));
    }

}
    