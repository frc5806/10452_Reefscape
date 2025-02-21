
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
// import com.revrobotics.CANSparkMax; (Depreciated)
// import com.revrobotics.CANSparkBase.ControlType; (Depreciated)
// import com.revrobotics.CANSparkBase.FaultID; (Depreciated)
// import com.revrobotics.SparkPIDController; (Depreciated)
// import com.revrobotics.CANSparkLowLevel.MotorType; (Depreciated)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.swerveCANcoderConfig;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANCoder absolute encoders.
 */
public class RevSwerveModule implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;

    private SparkMaxConfig angleConfig;
    private SparkMaxConfig driveConfig;

    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;


    // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Motor Config */
        // mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless); (Depreciated)
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        configAngleMotor();

        /* Drive Motor Config */
        // mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless); (Depreciated)
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        driveConfig = new SparkMaxConfig();
        configDriveMotor();

        /* Angle Encoder Config */

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();


        lastAngle = getState().angle;
    }


    private void configEncoders()
    {

        /* User can change the configs if they want, or leave it empty for factory-default */
        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);
        angleEncoder.getPosition().setUpdateFrequency(100);
        angleEncoder.getVelocity().setUpdateFrequency(100);

        // Depreciated
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        // Depreciated
        // relDriveEncoder.setPositionConversionFactor(Constants.Swerve.driveRevToMeters);
        // relDriveEncoder.setVelocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);
        driveConfig.encoder.positionConversionFactor(Constants.Swerve.driveRevToMeters);
        driveConfig.encoder.velocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);

        // Depreciated
        relAngleEncoder = mAngleMotor.getEncoder();
        // relAngleEncoder.setPositionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        // relAngleEncoder.setVelocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);

        angleConfig.encoder.positionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        angleConfig.encoder.velocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);

        synchronizeEncoders();
        // mDriveMotor.burnFlash(); (Depreciated)
        // mAngleMotor.burnFlash(); (Depreciated)
        mDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mAngleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configAngleMotor()
    {
        // Depreciated
        // mAngleMotor.restoreFactoryDefaults();
        // SparkPIDController controller = mAngleMotor.getPIDController();
        // controller.setP(Constants.Swerve.angleKP, 0);
        // controller.setI(Constants.Swerve.angleKI,0);
        // controller.setD(Constants.Swerve.angleKD,0);
        // controller.setFF(Constants.Swerve.angleKFF,0);
        // controller.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        // mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        angleConfig.closedLoop.p(Constants.Swerve.angleKP, ClosedLoopSlot.kSlot0);
        angleConfig.closedLoop.i(Constants.Swerve.angleKI, ClosedLoopSlot.kSlot0);
        angleConfig.closedLoop.d(Constants.Swerve.angleKD, ClosedLoopSlot.kSlot0);
        angleConfig.closedLoop.velocityFF(Constants.Swerve.angleKFF, ClosedLoopSlot.kSlot0);
        angleConfig.closedLoop.outputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        angleConfig.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        // Depreciated
        // mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        // mAngleMotor.setClosedLoopRampRate(Constants.Swerve.angleRampRate);

        angleConfig.inverted(Constants.Swerve.angleMotorInvert);
        angleConfig.idleMode(Constants.Swerve.angleIdleMode);
        angleConfig.closedLoopRampRate(Constants.Swerve.angleRampRate);

    }

    private void configDriveMotor()
    {
        // Depreciated
        // mDriveMotor.restoreFactoryDefaults();
        // SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        // controller.setP(Constants.Swerve.driveKP,0);
        // controller.setI(Constants.Swerve.driveKI,0);
        // controller.setD(Constants.Swerve.driveKD,0);
        // controller.setFF(Constants.Swerve.driveKFF,0);
        // controller.setOutputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        // mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        // mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        // mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);

        driveConfig.closedLoop.p(Constants.Swerve.driveKP, ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.i(Constants.Swerve.driveKI, ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.d(Constants.Swerve.driveKD, ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.velocityFF(Constants.Swerve.driveKFF, ClosedLoopSlot.kSlot0);
        driveConfig.closedLoop.outputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        driveConfig.smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveConfig.inverted(Constants.Swerve.driveMotorInvert);
        driveConfig.idleMode(Constants.Swerve.driveIdleMode);
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, isOpenLoop);

        // if(mDriveMotor.getFault(FaultID.kSensorFault)) (Depreciated)
        if(mDriveMotor.getFaults().sensor)
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        // if(mAngleMotor.getFault(FaultID.kSensorFault)) (Depreciated)
        if(mAngleMotor.getFaults().sensor)
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        // controller.setReference(velocity, ControlType.kVelocity, 0); (Depreciated)
        controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    }

    private void setAngle(SwerveModuleState desiredState)
    {
        SmartDashboard.putNumber("desiredState", desiredState.speedMetersPerSecond);
        if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        {
            mAngleMotor.stopMotor();
            return;

        }


        Rotation2d angle = desiredState.angle;
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        double degReference = angle.getDegrees();

        SmartDashboard.putNumber("Angle", angle.getDegrees());
        SmartDashboard.putNumber("Reference", degReference);

        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();

        // controller.setReference(degReference, ControlType.kPosition, 0); (Depreciated)
        controller.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder()
    {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        //return getAngle();
    }

    public int getModuleNumber()
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    public void synchronizeEncoders()
    {
       double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees(); 
       relAngleEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public double getOmega()
    {
        return angleEncoder.getVelocity().getValueAsDouble()/360;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle()
        );
    }
}
