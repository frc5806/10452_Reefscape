package frc.robot.commands.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveBase;


public class TimedDrive extends Command {
    private SwerveBase Swerve;
    private double x;
    private double y;
    private double rotation;
    protected Timer m_timer = new Timer();

    private final double m_duration;

    public TimedDrive(SwerveBase Swerve, double time, double x, double y, double rotation) {
        this.Swerve = Swerve;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        m_duration = time;
        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        Swerve.drive(
            new Translation2d(x, y),
            rotation,
            false, 
            true
        );
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
        m_timer.stop();

        Swerve.drive(
            new Translation2d(0, 0),
            0,
            false, 
            true
        );
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_duration);
    }
}