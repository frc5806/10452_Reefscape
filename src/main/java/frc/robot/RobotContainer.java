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
import frc.robot.auto.Auto;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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
    private final XboxController2 controller = new XboxController2(0);
    private final XboxController2 controller2 = new XboxController2(1);
    /* Drive Controls */
    private final double speedMod = 0.25; // Set back to 1
    // private boolean precisionMode = false;
    /* Subsystems */ 
    private final SwerveBase s_Swerve = new SwerveBase();
    // private final Intake intake = new Intake();
    // private final UsbCamera camera;
    // private final Servo linearActuator = new Servo(1);
    private final LinearServo coralServo = new LinearServo(1);
    private final LinearServo algaeServo = new LinearServo(0);
    private SparkMax coralMotor = new SparkMax(13, MotorType.kBrushless);
    private SparkMax algaeMotor = new SparkMax(12, MotorType.kBrushless);

    // private SparkMax elevatorMotor1 = new SparkMax(2, MotorType.kBrushless);
    // private SparkMax elevatorMotor2 = new SparkMax(3, MotorType.kBrushless);
    private SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private SparkMaxConfig coralConfig = new SparkMaxConfig();

    private final Elevator elevator = new Elevator();

    private final Vision vision = new Vision();

    /* Commands */
    // private final Command AUTO_Path = new Auto(s_Swerve, shooter, intake).Path();
    // private final Command AUTO_TwoRing = new Auto(s_Swerve, shooter, intake).TwoRingAuto();
    // private final Command AUTO_full = new Auto(s_Swerve, shooter, intake).fullAuto();
    // private final Command AUTO_SingleRing = new Auto(s_Swerve, shooter, intake).SingleRing();

    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(15.01, 1.52, new Rotation2d(0)),
            false
    );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      //  NamedCommands.registerCommand("Intake", intake.runIntake(0.9).withTimeout(3));

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

        // Configure the button bindings
        elevatorConfig.idleMode(IdleMode.kBrake);
        coralConfig.idleMode(IdleMode.kBrake);

        // elevatorMotor1.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // elevatorMotor2.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configureDefaultCommands();
        configureButtonBindings();
    }

    public void configureDefaultCommands(){
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -controller.getLeftY() * speedMod,
                () -> -controller.getLeftX() * speedMod,
                () -> -controller.getRightX() * speedMod,
                () -> controller.y().getAsBoolean(),//controller.getRawButtonPressed(XboxController.Button.kY.value),
                () -> false
            )
        );

        // elevator.setDefaultCommand(
        //     new ElevatorHardstop(elevator, () -> -controller2.getLeftY())
        //   );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
//--------------------------------------- Controller 1 ----------------------------------------------------------
        controller.b().onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0,0, new Rotation2d()))));

        // controller.a().onTrue(elevator.setElevatorPosition(-25));
        // controller.x().onTrue(elevator.setElevatorPosition(-2));
        controller.povUp().onTrue(elevator.setElevatorPosition(-9.25));
        controller.povLeft().onTrue(elevator.setElevatorPosition(-12));
        controller.povDown().onTrue(elevator.setElevatorPosition(-5));
        controller.povRight().onTrue(elevator.setElevatorPosition(0));
        controller.y().onTrue(new InstantCommand(() -> System.out.println(elevator.getEncoderPos())));
        controller.a().onTrue(elevator.setElevatorPosition(-16));
        // controller.b().onTrue(new InstantCommand(() -> elevator.resetEncoder()));
        // controller.a().onTrue(elevator.setElevatorPosition(0));

        // controller2.start().onTrue(new InstantCommand(() -> elevator.hardstop()));

        controller2.x().onTrue(new InstantCommand(() -> coralServo.set(0.85)));
        controller2.a().onTrue(new InstantCommand(() -> coralServo.set(0.25)));
        controller2.start().onTrue(new InstantCommand(() -> coralServo.set(0.15)));
        controller2.y().onTrue(new InstantCommand(() -> algaeServo.set(0.5)));
        controller2.b().onTrue(new InstantCommand(() -> algaeServo.set(0)));
    
        controller2.leftBumper().whileTrue(new InstantCommand(() -> coralMotor.set(0.4)));
        controller2.leftBumper().whileFalse(new InstantCommand(() -> coralMotor.set(0)));
        controller2.leftTrigger().whileTrue(new InstantCommand(() -> coralMotor.set(-0.4)));
        controller2.leftTrigger().whileFalse(new InstantCommand(() -> coralMotor.set(0)));


        controller2.rightBumper().whileTrue(new InstantCommand(() -> algaeMotor.set(0.6)));
        controller2.rightBumper().whileFalse(new InstantCommand(() -> algaeMotor.set(0)));
        controller2.rightTrigger().whileTrue(new InstantCommand(() -> algaeMotor.set(-0.6)));
        controller2.rightTrigger().whileFalse(new InstantCommand(() -> algaeMotor.set(0)));



        // controller.a()

        // controller2.povDown().whileTrue(new InstantCommand(() -> {elevatorMotor1.set(0.1); elevatorMotor2.set(-0.1);}));
        // controller2.povUp().whileTrue(new InstantCommand(() -> {elevatorMotor1.set(-0.1); elevatorMotor2.set(0.1);}));


        // controller.y().onTrue(new InstantCommand(() -> System.out.println(coralServo.get())));

        // controller.x().whileTrue(intake.runIntake(-0.8));
        // controller.y().whileTrue(intake.runIntake(1));

        // autoMove.whileTrue(autoMoveCommand);
        // autoMove.toggleOnFalse(new InstantCommand(() -> autoMoveCommand.cancel()));
     
        //controller.leftTrigger(0.01).whileTrue(new TeleopIntake(intake, () ->controller.getLeftTriggerAxis()));
        //controller.rightTrigger(0.01).whileTrue(new TeleopIntake(intake, () -> -controller.getRightTriggerAxis()));

        // controller.leftBumper().whileTrue(elevator.runElevator(0.5));
        // controller.rightBumper().whileTrue(elevator.runElevator(-0.5));


//------------------------------------ Controller 2 -------------------------------------------------

        // Shooter
        // controller2.x().whileTrue(shooter.runShooter(1));
        // controller2.a().whileTrue(new FireShooter(shooter, 4900));     

        // Amp
        // controller2.leftTrigger(0.01).whileTrue(shooter.runShooter(-1));
        // controller2.b().whileTrue(shooter.runAmp(-0.7));
        // controller2.y().whileTrue(shooter.runAmp(0.75));
        // controller2.a().whileTrue(shooter.runAmp(0.3));

        // controller2.b().whileTrue(new LoadAmp(shooter, () -> -0.3, () -> shooter.beamBroken()));

        // Intake
        // controller2.leftBumper().whileTrue(intake.runIntake(-0.8));
        // controller2.rightBumper().whileTrue(intake.runIntake(0.8));
        // controller2.rightTrigger(0.01).whileTrue(new AutoIntake(intake, () -> 0.5));


        // Elevator
        // controller2.povDown().onTrue(new InstantCommand(() -> elevator.elevatorHardstop()));
        // controller2.povUp().onTrue(new InstantCommand(() -> elevator.resetElevator()));
        // controller2.povRight().whileTrue(elevator.runMotor1(0.3));
        // controller2.povLeft().whileTrue(elevator.runMotor2(0.3));
        //controller2.x().whileTrue(new ElevatorPosition(100, elevator));
    
    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Red1");
        // Run path
        // try {
        //     return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Straight"));
        // } catch (Exception e) {
        //     return Commands.none();
        // }
    //     return AUTO_full;
    }

}
 