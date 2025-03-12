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
    private final SparkMaxConfig climbConfig;

    public Climb() {
        climbMotor = new SparkMax(14, MotorType.kBrushless);
        climbConfig = new SparkMaxConfig();
        configClimb();
    }

    private void configClimb() {
        climbConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command climbMotor(double speed) {
        return run(
            () -> { climbMotor.set(speed); }
        );
    }
}