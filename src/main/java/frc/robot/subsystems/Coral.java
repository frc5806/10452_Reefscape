package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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

    public Command coralMotorAutonomous(double speed) {
        Command autoCommand = new Command() {
            private boolean reached_speed = false;
            public void execute() {
                coralMotor.set(speed);
            }
 
            public boolean isFinished() {
                if (coralMotor.getEncoder().getVelocity() > 2000 && speed > 0) {
                    reached_speed = true;
                }
    
                if (coralMotor.getEncoder().getVelocity() < -2000 && speed < 0) {
                    return true;
                } else if (coralMotor.getEncoder().getVelocity() < 1000 && reached_speed && speed > 0) {
                    return true;
                }
                return false;
            }

            public void end(boolean interrupted) {
                coralMotor.set(0);
            }
        };

        return autoCommand;
    }
}