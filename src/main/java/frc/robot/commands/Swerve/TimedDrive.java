package frc.robot.commands.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveBase;


public class TimedDrive extends WaitCommand {
    private SwerveBase Swerve;
    private double x;
    private double y;
    private double rotation;

    public TimedDrive(SwerveBase Swerve, double time, double x, double y, double rotation) {
        super(time);
        this.Swerve = Swerve;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        // Swerve.drive(
        //     new Translation2d(x, y),
        //     rotation,
        //     false, 
        //     true
        // );
    }

    @Override
    public void execute() {
        Swerve.drive(
            new Translation2d(x, y),
            rotation,
            false, 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.drive(
            new Translation2d(0, 0),
            0,
            false, 
            true
        );
    }
}