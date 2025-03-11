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
        algaeServo1 = new LinearServo(0);
        algaeServo2 = new LinearServo(2);
        algaeMotor = new SparkMax(12, MotorType.kBrushless);
        algaeConfig = new SparkMaxConfig();
        configAlgae();
    }

    private void configAlgae() {
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command algaeServo(double position) {
        return run(
            () -> { algaeServo1.set(position); algaeServo2.set(position); }
        );
    }

    public Command algaeMotor(double speed) {
        return run(
            () -> { algaeMotor.set(speed); }
        );
    }
}