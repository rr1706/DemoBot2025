package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Aziumth;
import frc.robot.Constants.ModuleConstants.Drive;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class SwerveModule extends SubsystemBase{
    private final SparkMax m_azimuthMotor;
    private final SparkMax m_driveMotor;
    private final SparkMaxConfig m_motorConfigAz = new SparkMaxConfig();
    private final SparkClosedLoopController m_azimuthPID;
    private final SparkClosedLoopController m_drivePID;
    public final AbsoluteEncoder m_azimuthEnc; //set in robot container
    public final RelativeEncoder m_driveEnc; // set in robot container, use rev setup, encoder is apart of motorconfig
    private final SparkMaxConfig m_motorConfigDrive = new SparkMaxConfig();
    private final AbsoluteEncoderConfig m_azimuthEncConfig;
     

    public SwerveModule(int moduleID, double offset) {
        m_azimuthMotor = new SparkMax(moduleID+1, MotorType.kBrushless);
        m_azimuthPID = m_azimuthMotor.getClosedLoopController();
        m_motorConfigAz
            .smartCurrentLimit(CurrentLimit.kAzimuth)
            .voltageCompensation(GlobalConstants.kVoltageCompensation)
            .idleMode(IdleMode.kBrake);
        m_azimuthEncConfig = new AbsoluteEncoderConfig();
        m_azimuthEncConfig
            .positionConversionFactor(Aziumth.kPositionFactor)
            .velocityConversionFactor(Aziumth.kVelocityFactor);
        m_motorConfigAz.absoluteEncoder.apply(m_azimuthEncConfig);
            
        m_azimuthEnc = m_azimuthMotor.getAbsoluteEncoder();
        m_motorConfigAz.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(Aziumth.kp)
            .positionWrappingInputRange(0.0, 2*Math.PI)
            .positionWrappingEnabled(true);
        m_azimuthMotor.configure(m_motorConfigAz, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        m_driveMotor = new SparkMax(moduleID, MotorType.kBrushless);
        m_drivePID = m_driveMotor.getClosedLoopController();
        m_motorConfigDrive
            .smartCurrentLimit(CurrentLimit.kDrive)
            .voltageCompensation(GlobalConstants.kVoltageCompensation)
            .idleMode(IdleMode.kBrake); 
        m_driveEnc = m_driveMotor.getEncoder();
        m_motorConfigDrive.encoder
            .positionConversionFactor(Drive.kGearRatio)
            .velocityConversionFactor(Drive.kGearRatio/60);
        m_motorConfigDrive.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Drive.kp)
                .velocityFF(Drive.kf);
        m_driveMotor.configure(m_motorConfigDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getStateAngle()));
    }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getStateAngle()));
    }
    public double getDriveVelocity() {
        return m_driveEnc.getVelocity();
    }

    public double getDrivePosition() {
        return m_driveEnc.getPosition();
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));

        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        m_azimuthPID.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public double getStateAngle() {
        return m_azimuthEnc.getPosition();
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_azimuthMotor.stopMotor();
    }
}
