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

    public Command coralMotor(double speed) {
        return run(
            () -> { 

                //We should output this value to see what is going on
                if(coralMotor.getEncoder().getVelocity() < 1){
                    coralMotor.set(speed);
                }

            }
        );
    }
}