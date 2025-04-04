package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Algae extends SubsystemBase {
    private final LinearServo algaeServo1;
    private final LinearServo algaeServo2;
    private final SparkMax algaeMotor;
    private final SparkMaxConfig algaeConfig;

    public Algae() {
        //Identify each part of the algae system
        algaeServo1 = new LinearServo(0);
        algaeServo2 = new LinearServo(2);
        algaeMotor = new SparkMax(12, MotorType.kBrushless);
        algaeConfig = new SparkMaxConfig();
        configAlgae();
    }

    //configure algae motor
    private void configAlgae() {
        //Has to be braking so can hold coral there
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //Set the algae linear actuator position
    public Command algaeServo(double position) {
        return run(
            () -> { algaeServo1.set(position); algaeServo2.set(position); }
        );
    }

    //Command to run the algaemotor at a set speed
    public Command algaeMotor(double speed) {
        return run(
            () -> { algaeMotor.set(speed); }
        );
    }


    //Algae motor command for autonomous (needs to have an "end" state)
    public Command algaeMotorAutonomous(double speed) {
        Command autoCommand = new Command() {
            private boolean reached_speed = false;
            public void execute() {
                algaeMotor.set(speed);
            }
 
            public boolean isFinished() {
                if (algaeMotor.getEncoder().getVelocity() > 2000 && speed > 0) {
                    reached_speed = true;
                }
    
                if (algaeMotor.getEncoder().getVelocity() < -2000 && speed < 0) {
                    return true;
                } else if (algaeMotor.getEncoder().getVelocity() < 1000 && reached_speed && speed > 0) {
                    return true;
                }
                return false;
            }

            public void end(boolean interrupted) {
                algaeMotor.set(0);
            }
        };

        return autoCommand;
    }

    //Algae method for autonomous
    public Command algaeServoAutonomous(double position) {
        Command autoCommand = new Command() {
            public void initialize() {
                algaeServo1.set(position);
                algaeServo2.set(position);
            }
 
            public boolean isFinished() {
                if (Math.abs(algaeServo1.get() - position) < 0.1) {
                    return true;
                }
                return false;
            }

            public void end(boolean interrupted) {
            }
        };

        return autoCommand;
    }
}