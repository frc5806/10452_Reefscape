package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

// import com.revrobotics.CANSparkMax; (Depreciated)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkLowLevel.MotorType; (Depreciated)
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotor1;
    private final SparkMax elevatorMotor0;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config0;
    private final SparkMaxConfig config1;
    private final SparkClosedLoopController elevatorPID;

    private boolean hardstop = true;
    private double global_setpoint = 0;

    public Elevator() {
        elevatorMotor0 = new SparkMax(ElevatorConstants.kElevatorMotorPort0, MotorType.kBrushless);
        elevatorMotor1 = new SparkMax(ElevatorConstants.kElevatorMotorPort1, MotorType.kBrushless);

        config0 = new SparkMaxConfig();
        config1 = new SparkMaxConfig();
        configElevator();

        encoder = elevatorMotor0.getEncoder();
        resetEncoder();

        elevatorPID = elevatorMotor0.getClosedLoopController();

    }

    public void configElevator() {
        config0.closedLoop.p(Constants.ElevatorConstants.elevatorKP, ClosedLoopSlot.kSlot0);
        config0.closedLoop.i(Constants.ElevatorConstants.elevatorKI, ClosedLoopSlot.kSlot0);
        config0.closedLoop.d(Constants.ElevatorConstants.elevatorKD, ClosedLoopSlot.kSlot0);
        config0.closedLoop.outputRange(-Constants.ElevatorConstants.elevatorPower,
                Constants.ElevatorConstants.elevatorPower);
        config0.smartCurrentLimit(Constants.ElevatorConstants.elevatorContinuousCurrentLimit);
        config0.inverted(Constants.ElevatorConstants.elevatorInverted);
        config0.idleMode(Constants.ElevatorConstants.elevatorIdleMode);

        elevatorMotor0.configure(config0, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config1.follow(Constants.ElevatorConstants.kElevatorMotorPort0, false);
        elevatorMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double pwr) {
        elevatorMotor0.set(pwr);
    }

    public void setPosition(double pos) {
        elevatorPID.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Command setElevatorPosition(double setpoint) {
        // If the elevator is set to the bottom, turn the motor off
        this.global_setpoint = setpoint;

        if (setpoint == 0) {

            // return new Command() {
            // private boolean isfinished = false;
            // public void initialize(){
            // elevatorMotor0.set(0); elevatorMotor1.set(0);
            // isfinished = true;
            // }
            // public boolean isFinished(){
            // return isfinished;
            // }
            // };
            return run(
                    () -> {
                        elevatorMotor0.set(0);
                        elevatorMotor1.set(0);
                    });
        }

        // return new Command(){
        // public void execute(){
        // elevatorPID.setReference(setpoint, ControlType.kPosition,
        // ClosedLoopSlot.kSlot0);

        // }
        // public boolean isFinished(){
        // return getEncoderPos() == setpoint;
        // }
        // };

        return run(
                () -> {
                    elevatorPID.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                });

    }

    public Command setElevatorPositionAutonomous(double setpoint) {
        Command autoCommand = new Command() {
            public void initialize() {
                setElevatorPosition(setpoint).schedule();
            }
 
            public boolean isFinished() {
                double threshold = (-2/10) * (setpoint+6) + 3.5;
                return Math.abs(getEncoderPos() - setpoint) < threshold;
            }

            public void end(boolean interrupted) {
                // I don't think we need to do anything here
            }
        };

        return autoCommand;

    }

    public double getEncoderPos() {
        return encoder.getPosition();
    }

    public DoubleSupplier getEncoderPosSupplier() {
        return () -> {
            return encoder.getPosition();
        };
    }

    public void resetEncoder() {
        // Only allow the encoder to be reset if the global setpoint is -2
        if (this.global_setpoint == 0) {
            encoder.setPosition(0);
        }
    }

    public void hardstop() {
        this.hardstop = !hardstop;
    }

    public boolean getHardstop() {
        return this.hardstop;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getEncoderPos());
        SmartDashboard.putBoolean("Elevator Hardstop", hardstop);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}