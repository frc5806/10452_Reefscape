package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.XboxController2;
/* Subsystem Imports */
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Limelight.LimelightData;
import frc.robot.Constants.Swerve;
/* Command Imports */
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Swerve.AlignLimelightReef;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.Swerve.TimedDrive;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    /* Controllers */
    private final XboxController2 driverController = new XboxController2(0);
    private final XboxController2 operatorController = new XboxController2(1);
    /* Drive Controls */
    private final double translationSpeedMod = 0.75;
    private final double rotationSpeedMod = 0.5;
    /* Subsystems */
    private final SwerveBase s_Swerve = new SwerveBase();
    private final Elevator elevator = new Elevator();
    private final Coral coral = new Coral();
    private final Algae algae = new Algae();
    private final Climb climb = new Climb();
    // private final Vision vision = new Vision();
    private final Limelight limelight = new Limelight();

    SequentialCommandGroup L3FullAuto = new SequentialCommandGroup(
        elevator.setElevatorPositionAutonomous(-13.25),
        coral.coralServoAutonomous(0.25),
        new WaitCommand(3),
        coral.coralMotorTimed(-0.4, 0.5),
        elevator.setElevatorPositionAutonomous(0),
        coral.coralServoAutonomous(0.82),
        new InstantCommand(() -> System.out.println("done! L3"))
    );

    SequentialCommandGroup L2FullAuto = new SequentialCommandGroup(
        elevator.setElevatorPositionAutonomous(-6.25),
        coral.coralServoAutonomous(0.25),
        new WaitCommand(3),
        coral.coralMotorTimed(-0.4, 0.5),
        elevator.setElevatorPositionAutonomous(0),
        coral.coralServoAutonomous(0.82),
        new InstantCommand(() -> System.out.println("done! L2"))
    );

    SequentialCommandGroup AlgaeL3FullAuto = new SequentialCommandGroup(
        elevator.setElevatorPositionAutonomous(-17),
        algae.algaeServoAutonomous(0.8),
        new WaitCommand(4),
        algae.algaeMotorTimed(-0.6, 1),
        elevator.setElevatorPositionAutonomous(0),
        algae.algaeServoAutonomous(0)
    );

    SequentialCommandGroup AlgaeL2FullAuto = new SequentialCommandGroup(
        elevator.setElevatorPositionAutonomous(-10.5),
        algae.algaeServoAutonomous(0.8),
        new WaitCommand(4),
        algae.algaeMotorTimed(-0.6, 1),
        elevator.setElevatorPositionAutonomous(0),
        algae.algaeServoAutonomous(0)
    );

    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(15.01, 1.52, new Rotation2d(0)),
            false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Networking
        PortForwarder.add(5800, "10.75.58.06", 5800);
        PortForwarder.add(1181, "10.75.58.06", 1181);

        // vision.startVision(); // Webcam

        configureDefaultCommands();
        configureButtonBindings();

        // LimelightData.setOn();
    }

    public void configureDefaultCommands() {
        //Configure bindings for serve
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverController.getLeftY() * translationSpeedMod,
                        () -> -driverController.getLeftX() * translationSpeedMod,
                        () -> -driverController.getRightX() * rotationSpeedMod,
                        () -> driverController.y().getAsBoolean(),
                        () -> false));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Controller */
        // Climb in
        driverController.b().whileTrue(climb.climbMotor(0.1, 0));
        driverController.b().onFalse(climb.climbMotor(0, 0));

        // Climb out
        driverController.y().whileTrue(climb.climbMotor(-0.1, 0));
        driverController.y().onFalse(climb.climbMotor(0, 0));

        // Climb in
        driverController.a().whileTrue(climb.climbMotor(0, -0.7));
        driverController.a().onFalse(climb.climbMotor(0, 0));

        // Climb out
        driverController.x().whileTrue(climb.climbMotor(0, 0.7));
        driverController.x().onFalse(climb.climbMotor(0, 0));
        // driverController.b().onTrue(new InstantCommand(() -> elevator.resetEncoder()));

        // driverController.y().onTrue(new InstantCommand(() -> System.out.println(elevator.getEncoderPos())));

        // Align to algae on Reef
        driverController.back().whileTrue(new AlignLimelightReef(s_Swerve, -0.03, 0.4, true));
        // driverController.back().onTrue(coral.coralMotorTimed(0.4, 0.5));

        driverController.start().onTrue(new TimedDrive(s_Swerve, 0.5, 0.25, 0, 0));

        //Run elevator with POV and bumpers
        driverController.rightBumper().onTrue(elevator.setElevatorPosition(-17)); // Algae 3
        driverController.leftBumper().onTrue(elevator.setElevatorPosition(-10.5)); // Algae 2
        
        // Reset swerve odometry
        driverController.povUp().onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

        driverController.povLeft().onTrue(elevator.setElevatorPosition(-13)); // Coral 3
        driverController.povDown().onTrue(elevator.setElevatorPosition(-6)); // Coral 2
        driverController.povRight().onTrue(elevator.setElevatorPosition(0)); // Coral 1

        // Align limelight to april tag
        // TODO: We can correct alignment by having the robot stop further back (say 0.4 instead of 0.3), then driving forward with TimedDrive.
        driverController.rightTrigger().whileTrue(new AlignLimelightReef(s_Swerve, -0.12, 0.4, true)); // Left
        driverController.leftTrigger().whileTrue(new AlignLimelightReef(s_Swerve, 0.2, 0.4, true)); // Right

        /* Operator Controller */
        operatorController.a().onTrue(coral.coralServo(0.25)); // Coral down

        operatorController.x().onTrue(coral.coralServo(0.82)); // Coral up
        
        operatorController.b().onTrue(algae.algaeServo(0)); // Algae down
        
        operatorController.y().onTrue(algae.algaeServo(0.85)); // Algae up

        // Coral in
        operatorController.rightBumper().whileTrue(coral.coralMotor(0.4));
        operatorController.rightBumper().whileFalse(coral.coralMotor(0));
    
        // Coral out
        operatorController.rightTrigger().whileTrue(coral.coralMotor(-0.4));
        operatorController.rightTrigger().whileFalse(coral.coralMotor(0));

        // Algae in
        operatorController.leftTrigger().whileTrue(algae.algaeMotor(0.8));
        operatorController.leftTrigger().whileFalse(algae.algaeMotor(0));

        // Algae out
        operatorController.leftBumper().whileTrue(algae.algaeMotor(-0.6));
        operatorController.leftBumper().whileFalse(algae.algaeMotor(0));

        operatorController.start().onTrue(L3FullAuto); // Algae intake
        operatorController.back().onTrue(L2FullAuto); // Algae intake
        operatorController.povUp().onTrue(AlgaeL3FullAuto);
        operatorController.povDown().onTrue(AlgaeL2FullAuto);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // Limelight autonomous commands
        Command reef_align_left = new AlignLimelightReef(s_Swerve, 0.2, 0.3, false);
        Command reef_align_right = new AlignLimelightReef(s_Swerve, -0.12, 0.3, false);
        NamedCommands.registerCommand("reefAlignLimelightLeft", reef_align_left);
        NamedCommands.registerCommand("reefAlignLimelightRight", reef_align_right);

        // Elevator autonomous commands
        Command elevator_down = elevator.setElevatorPositionAutonomous(0);
        Command elevator_l2_coral = elevator.setElevatorPositionAutonomous(-6);
        Command elevator_l3_coral = elevator.setElevatorPositionAutonomous(-13);
        Command elevator_bottom_algae = elevator.setElevatorPositionAutonomous(-9.75);
        Command elevator_top_algae = elevator.setElevatorPositionAutonomous(-16.5);
        NamedCommands.registerCommand("elevatorDown", elevator_down);
        NamedCommands.registerCommand("elevatorReefL2", elevator_l2_coral);
        NamedCommands.registerCommand("elevatorReefL3", elevator_l3_coral);
        NamedCommands.registerCommand("elevatorAlgaeBottom", elevator_bottom_algae);
        NamedCommands.registerCommand("elevatorAlgaeTop", elevator_top_algae); 

        // Coral autonomous commands
        Command coral_up = coral.coralServoAutonomous(0.82);
        Command coral_down = coral.coralServoAutonomous(0.25);
        Command coral_intake = coral.coralMotorAutonomous(0.4);
        Command coral_shoot = coral.coralMotorAutonomous(-0.4);
        NamedCommands.registerCommand("coralServoUp", coral_up);
        NamedCommands.registerCommand("coralServoDown", coral_down);
        NamedCommands.registerCommand("coralIntake", coral_intake);
        NamedCommands.registerCommand("coralShoot", coral_shoot);

        Command coral_intake_time = coral.coralMotorTimed(0.4, 1.5);
        Command coral_shoot_time = coral.coralMotorTimed(-0.4, 1.5);
        NamedCommands.registerCommand("coralIntakeTime", coral_intake_time);
        NamedCommands.registerCommand("coralShootTime", coral_shoot_time);

        NamedCommands.registerCommand("elevatorReefL3Score", L3FullAuto);
        NamedCommands.registerCommand("elevatorReefL2Score", L2FullAuto);

        Command algae_up = algae.algaeServoAutonomous(0.8);
        Command algae_down = algae.algaeServoAutonomous(0.25);
        Command algae_intake = algae.algaeMotorAutonomous(1);
        Command algae_shoot = algae.algaeMotorAutonomous(-1);
        NamedCommands.registerCommand("algaeServoUp", algae_up);
        NamedCommands.registerCommand("algaeServoDown", algae_down);
        NamedCommands.registerCommand("algaeIntake", algae_intake);
        NamedCommands.registerCommand("algaeShoot", algae_shoot);


        // Register a specific auto from PathPlanner (that uses the above named commands) as our routine to run
        PathPlannerAuto autoCommand = new PathPlannerAuto("FirstMovementOpposite");
        return autoCommand;
    }
}
