package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//Elevator subsystem
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
        //Instantiate elevator motors
        elevatorMotor0 = new SparkMax(ElevatorConstants.kElevatorMotorPort0, MotorType.kBrushless);
        elevatorMotor1 = new SparkMax(ElevatorConstants.kElevatorMotorPort1, MotorType.kBrushless);

        config0 = new SparkMaxConfig();
        config1 = new SparkMaxConfig();
        configElevator();

        //Instantiate encoder to keep track of elevator height 
        encoder = elevatorMotor0.getEncoder();
        resetEncoder();
        elevatorPID = elevatorMotor0.getClosedLoopController();
    }

    public void configElevator() {
        //Update elevator motor configs based stuff in the constants file - it makes more sense in configs than here
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
        //Set motor speed
        elevatorMotor0.set(pwr);
    }


    public Command setElevatorPosition(double setpoint) {
        
        this.global_setpoint = setpoint;
        
        // If the elevator is set to the bottom, turn the motor off
        if (setpoint == 0) {

            return run(
                    () -> {
                        elevatorMotor0.set(0);
                        elevatorMotor1.set(0);
                    });
        }

        //Otherwise run the motors using PID to get to the setpoint
        return run(
                () -> {
                    elevatorPID.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                });

    }


    public Command setElevatorPositionAutonomous(double setpoint) {
        //Same thing as the earlier setElevatorPosition() command just used for autonomous so it has a recognizable "end" that can be identified by path planner
        Command autoCommand = new Command() {
            public void initialize() {
                setElevatorPosition(setpoint).schedule();
            }
 
            //Check if finished based on some linear relationship experimentally determined between setpoint and encoder position
            public boolean isFinished() {
                double threshold = Constants.ElevatorConstants.thresholdFunction(setpoint);
                return Math.abs(getEncoderPos() - setpoint) < threshold*2;
            }

            public void end(boolean interrupted) {
                // I don't think we need to do anything here
            }
        };

        return autoCommand;

    }


    public double getEncoderPos() {
        //get's current height based on encoder
        return encoder.getPosition();
    }

    public DoubleSupplier getEncoderPosSupplier() {
        //Same thing as earlier just returning a method that returns the encoder position
        return () -> {
            return encoder.getPosition();
        };
    }

    public void resetEncoder() {
        // Only allow the encoder to be reset if the global setpoint is 0
        if (this.global_setpoint == 0) {
            encoder.setPosition(0);
        }
    }


    //Be able to hardstop the elevator - haven't used this yet but might come in handy
    public void hardstop() {
        this.hardstop = !hardstop;
    }

    public boolean getHardstop() {
        return this.hardstop;
    }

    @Override
    public void periodic() {
        //Update some values on smartdashboard
        SmartDashboard.putNumber("Elevator Encoder", getEncoderPos());
        SmartDashboard.putBoolean("Elevator Hardstop", hardstop);
    }

    //Have not implemented simulations yet
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}