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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Aziumth;
import frc.robot.Constants.ModuleConstants.Drive;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveModule extends SubsystemBase {
    private final SparkMax m_azimuthMotor;
    private final SparkMax m_driveMotor;
    private final SparkMaxConfig m_motorConfigAz = new SparkMaxConfig();
    private final SparkClosedLoopController m_azimuthPID;
    private final SparkClosedLoopController m_drivePID;
    private final double m_offset;
    private final AnalogEncoder m_analogEnc;
    private final RelativeEncoder m_driveEnc; // set in robot container, use rev setup, encoder is apart of motorconfig
    private final SparkMaxConfig m_motorConfigDrive = new SparkMaxConfig();
    private final PIDController m_aziPID = new PIDController(1.0, 0.0, 0.0);

    public SwerveModule(int azimuthID, int driveID, int encoderID, double offset) {
        m_azimuthMotor = new SparkMax(azimuthID, MotorType.kBrushless);
        m_azimuthPID = m_azimuthMotor.getClosedLoopController();
        m_motorConfigAz
                .smartCurrentLimit(CurrentLimit.kAzimuth)
                .voltageCompensation(GlobalConstants.kVoltageCompensation)
                .idleMode(IdleMode.kBrake);
        m_azimuthMotor.configure(m_motorConfigAz, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_analogEnc = new AnalogEncoder(encoderID);
        m_offset = offset;
        m_aziPID.enableContinuousInput(-Math.PI, Math.PI);

        m_driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        m_drivePID = m_driveMotor.getClosedLoopController();
        m_motorConfigDrive
                .smartCurrentLimit(CurrentLimit.kDrive)
                .voltageCompensation(GlobalConstants.kVoltageCompensation)
                .idleMode(IdleMode.kBrake);
        m_driveEnc = m_driveMotor.getEncoder();
        m_motorConfigDrive.encoder
                .positionConversionFactor(Drive.kToMeters)
                .velocityConversionFactor(Drive.kToMeters / 60.0);
        m_motorConfigDrive.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Drive.kp)
                .velocityFF(Drive.kf);
        m_driveMotor.configure(m_motorConfigDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getStateAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getStateAngle());
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
        desiredState.optimize(getStateAngle());

        m_drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        double pidOutput = m_aziPID.calculate(getStateAngle().getRadians(), desiredState.angle.getRadians());

        m_azimuthMotor.set(pidOutput);

    }

    public Rotation2d getStateAngle() {
        return new Rotation2d(m_analogEnc.get()*2*Math.PI+m_offset);
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_azimuthMotor.stopMotor();
    }


    
}
