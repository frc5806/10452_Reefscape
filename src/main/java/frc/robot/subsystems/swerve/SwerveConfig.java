package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.configs.SensorInitializationStrategy;
// import com.ctre.phoenix6.configs.SensorTimeBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import frc.robot.Constants;


public class SwerveConfig
{

    public CANcoderConfiguration canCoderConfig;
    public MotorOutputConfigs motorConfigs;
    public SensorDirectionValue sensorDirection;
    public MagnetSensorConfigs config;

    /* Swerve Profiling Values */
    public SwerveConfig() {
        canCoderConfig = new CANcoderConfiguration();
        motorConfigs = new MotorOutputConfigs();
        config = new MagnetSensorConfigs();
        sensorDirection = Constants.Swerve.canCoderInvert;
        config.withAbsoluteSensorDiscontinuityPoint(1);

        // canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}

