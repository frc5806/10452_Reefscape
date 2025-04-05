package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveBase;


public class Coral extends SubsystemBase {
    private final LinearServo coralServo;
    private final SparkMax coralMotor;
    private final SparkMaxConfig coralConfig;

    public Coral() {
        coralServo = new LinearServo(1);
        coralMotor = new SparkMax(13, MotorType.kBrushless);
        coralConfig = new SparkMaxConfig();
        configCoral();
    }

    private void configCoral() {
        coralConfig.idleMode(IdleMode.kBrake);
        coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command coralServo(double position) {
        return run(
            () -> { 
                
                    coralServo.set(position);
                
            }
        );
    }

    public Command coralServoAutonomous(double position) {
        Command autoCommand = new Command() {
            public void initialize() {
                coralServo.set(position);
            }
 
            public boolean isFinished() {
                if (Math.abs(coralServo.get() - position) < 0.1) {
                    return true;
                }
                return false;
            }

            public void end(boolean interrupted) {
            }
        };

        return autoCommand;
    }

    public Command coralMotor(double speed) {
        return run(
            () -> { 
                coralMotor.set(speed);
                // System.out.println(coralMotor.getEncoder().getVelocity());
            }
        );
    }

    public Command coralMotorTimed(double speed, double time) {
        class TimedCoral extends Command {
            private double speed;
            protected Timer m_timer = new Timer();

            private final double m_duration;

            public TimedCoral(double speed, double time) {
                this.speed = speed;
                m_duration = time;
            }

            @Override
            public void initialize() {
                m_timer.restart();

                coralMotor.set(speed);
            }

            @Override
            public void execute() {
                coralMotor.set(speed);
            }

            @Override
            public void end(boolean interrupted) {
                m_timer.stop();

                coralMotor.set(0);
            }

            @Override
            public boolean isFinished() {
                return m_timer.hasElapsed(m_duration);
            }
        }
        return new TimedCoral(speed, time);
    }

    public Command coralMotorAutonomous(double speed) {
        Command autoCommand = new Command() {
            private boolean reached_speed = false;
            public void execute() {
                coralMotor.set(speed);
            }
 
            public boolean isFinished() {
                //If intaking and reached max speed then we have reached max speed
                if (coralMotor.getEncoder().getVelocity() > 2000 && speed > 0) {
                    reached_speed = true;
                }
                
                //If shooting and have reached max speed (less than because is MORE negative) then done
                if (coralMotor.getEncoder().getVelocity() < -2000 && speed < 0) {
                    return true;

                //If intaking and have reached the maximum speed, and are now slowing down, then done
                } else if (coralMotor.getEncoder().getVelocity() < 1000 && reached_speed && speed > 0) {
                    return true;
                }
                //Otherwise continue!
                return false;
            }

            public void end(boolean interrupted) {
                coralMotor.set(0);
            }
        };

        return autoCommand;
    }
}