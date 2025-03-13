package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
// import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Auto;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Swerve.AlignLimelight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.lib.util.XboxController2;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.LinearServo;
import frc.robot.subsystems.Vision;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    /* Controllers */
    private final XboxController2 driverController = new XboxController2(0);
    private final XboxController2 operatorController = new XboxController2(1);
    /* Drive Controls */
    private final double translationSpeedMod = 0.75; // Set back to 1
    private final double rotationSpeedMod = 0.5; // Set back to 1
    // private boolean precisionMode = false;
    /* Subsystems */
    private final SwerveBase s_Swerve = new SwerveBase();
    // private final UsbCamera camera;
    // private final Servo linearActuator = new Servo(1);

    private final Elevator elevator = new Elevator();
    private final Coral coral = new Coral();
    private final Algae algae = new Algae();
    private final Climb climb = new Climb();

    private final Vision vision = new Vision();

    private final Limelight limelight = new Limelight();

    /* Commands */
    // private final Command AUTO_Path = new Auto(s_Swerve, shooter, intake).Path();
    // private final Command AUTO_TwoRing = new Auto(s_Swerve, shooter,
    // intake).TwoRingAuto();
    // private final Command AUTO_full = new Auto(s_Swerve, shooter,
    // intake).fullAuto();
    // private final Command AUTO_SingleRing = new Auto(s_Swerve, shooter,
    // intake).SingleRing();

    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(15.01, 1.52, new Rotation2d(0)),
            false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // NamedCommands.registerCommand("Intake",
        // intake.runIntake(0.9).withTimeout(3));

        /* Auto */
        // PathPlannerServer.startServer(5811);
        // movementChooser.setDefaultOption("taxi", new Taxi(s_Swerve));
        // movementChooser.addOption("No Movement", new InstantCommand());
        // SmartDashboard.putData("Movement", movementChooser);

        /* Networking */
        PortForwarder.add(5800, "10.75.58.06", 5800);
        PortForwarder.add(1181, "10.75.58.06", 1181);

        // camera = CameraServer.startAutomaticCapture(0);
        // camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        vision.startVision();

        configureDefaultCommands();
        configureButtonBindings();
    }

    public void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driverController.getLeftY() * translationSpeedMod,
                        () -> -driverController.getLeftX() * translationSpeedMod,
                        () -> -driverController.getRightX() * rotationSpeedMod,
                        () -> driverController.y().getAsBoolean(), // driverController.getRawButtonPressed(XboxController.Button.kY.value),
                        () -> false));

        // elevator.setDefaultCommand(
        // new ElevatorHardstop(elevator, () -> operatorController.getLeftY())
        // );
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
        // -------------------- Driver Controller --------------------
        driverController.rightTrigger().whileTrue(new AlignLimelight(s_Swerve, -0.12, 0.3));
        driverController.leftTrigger().whileTrue(new AlignLimelight(s_Swerve, 0.2, 0.3));

        driverController.rightBumper().onTrue(elevator.setElevatorPosition(-16.5));
        driverController.leftBumper().onTrue(elevator.setElevatorPosition(-9.75));
        
        driverController.povLeft().onTrue(elevator.setElevatorPosition(-14));
        driverController.povRight().onTrue(elevator.setElevatorPosition(0));
        driverController.povDown().onTrue(elevator.setElevatorPosition(-6));

        driverController.a().whileTrue(climb.climbMotor(0.6));
        driverController.a().whileFalse(climb.climbMotor(0));

        driverController.x().whileTrue(climb.climbMotor(-0.6));
        driverController.x().whileFalse(climb.climbMotor(0));

        driverController.b().onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));
        // driverController.b().onTrue(new InstantCommand(() -> elevator.resetEncoder()));

        driverController.y().onTrue(new InstantCommand(() -> System.out.println(elevator.getEncoderPos())));

        // -------------------- Operator Controller --------------------
        operatorController.x().onTrue(coral.coralServo(0.8));
        
        operatorController.a().onTrue(coral.coralServo(0.25));
        
        operatorController.y().onTrue(algae.algaeServo(0.7));

        operatorController.b().onTrue(algae.algaeServo(0));

        operatorController.leftBumper().whileTrue(coral.coralMotor((0.4)));
        operatorController.leftBumper().whileFalse(coral.coralMotor((0)));
        operatorController.leftTrigger().whileTrue(coral.coralMotor((-0.4)));
        operatorController.leftTrigger().whileFalse(coral.coralMotor((0)));

        operatorController.rightBumper().whileTrue(algae.algaeMotor((0.6)));
        operatorController.rightBumper().whileFalse(algae.algaeMotor((0)));
        operatorController.rightTrigger().whileTrue(algae.algaeMotor((-1)));
        operatorController.rightTrigger().whileFalse(algae.algaeMotor((0)));
    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command align_left = new AlignLimelight(s_Swerve, 0.2, 0.3);
        Command align_right = new AlignLimelight(s_Swerve, -0.12, 0.3);
        NamedCommands.registerCommand("alignLimelightLeft", align_left);
        NamedCommands.registerCommand("alignLimelightRight", align_right);

        Command elevator_down = elevator.setElevatorPosition(0);
        Command elevator_l2 = elevator.setElevatorPosition(-6);
        Command elevator_l3 = elevator.setElevatorPosition(-14);
        NamedCommands.registerCommand("elevatorDown", elevator_down);
        NamedCommands.registerCommand("elevatorReefL2", elevator_l2);
        NamedCommands.registerCommand("elevatorReefL3", elevator_l3);
    
        Command coral_up = coral.coralServo(0.8);
        Command coral_down = coral.coralServo(0.25);
        Command coral_intake = coral.coralMotor(0.4);
        Command coral_shoot = coral.coralMotor(-0.4);
        NamedCommands.registerCommand("coralServoUp", coral_up);
        NamedCommands.registerCommand("coralServoDown", coral_down);
        NamedCommands.registerCommand("coralIntake", coral_intake);
        NamedCommands.registerCommand("coralShoot", coral_shoot);







        PathPlannerAuto autoCommand = new PathPlannerAuto("PracticeAuto");
        return autoCommand;
        // Run path
        // try {
        // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Straight"));
        // } catch (Exception e) {
        // return Commands.none();
        // }
        // return AUTO_full;
    }

}
