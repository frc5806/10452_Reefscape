package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private final SparkMax winchMotor1;
    private final SparkMax winchMotor2;
    private final SparkMaxConfig climbConfig;
    private final SparkMaxConfig winchConfig1;
    private final SparkMaxConfig winchConfig2;

    public Climb() {
        climbMotor = new SparkMax(14, MotorType.kBrushless);
        winchMotor1 = new SparkMax(15, MotorType.kBrushless);
        winchMotor2 = new SparkMax(16, MotorType.kBrushless);
        climbConfig = new SparkMaxConfig();
        winchConfig1 = new SparkMaxConfig();
        winchConfig2 = new SparkMaxConfig();
        configClimb();
    }

    private void configClimb() {
        climbConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        winchConfig1.idleMode(IdleMode.kBrake);
        winchMotor1.configure(winchConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        winchConfig2.idleMode(IdleMode.kBrake);
        winchConfig2.inverted(true);
        winchMotor2.configure(winchConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command climbMotor(double climbSpeed, double winchSpeed) {
        return run(
            () -> {
                climbMotor.set(climbSpeed); 
                winchMotor1.set(winchSpeed);
                winchMotor2.set(winchSpeed);
            }
        );
    }
}