package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

// import com.revrobotics.CANSparkMax; (Depreciated)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkLowLevel.MotorType; (Depreciated)
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //private final CANSparkMax intakeMotor1; (Depreciated)
    private final SparkMax intakeMotor1; 
    private final RelativeEncoder encoder;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensorV3;
    
    
    public Intake() {
        //intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotorPort1, MotorType.kBrushless); (Depreciated)
        intakeMotor1 = new SparkMax(IntakeConstants.kIntakeMotorPort1, MotorType.kBrushless);

        encoder = intakeMotor1.getEncoder();
        resetEncoder();

        colorSensorV3 = new ColorSensorV3(i2cPort);
        
        
    }

    public Command runIntake(double pwr) {
        return this.startEnd(() -> this.set(pwr), () -> this.set(0));
    }

    public void set(double pwr) {
        intakeMotor1.set(pwr);
    }

    public double getIntakeEncoderPos(){
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public boolean isOrange() {
        return (colorSensorV3.getColor() == Color.kOrange);
    }

    public Color getColor() {
        return colorSensorV3.getColor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Orange", isOrange());
        SmartDashboard.putString("Color", getColor().toString());
        SmartDashboard.putBoolean("CS is Connected:", colorSensorV3.isConnected());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
}