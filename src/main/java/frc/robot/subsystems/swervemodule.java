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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class swervemodule extends SubsystemBase{
    private final SparkMax m_azimuthMotor;
    private final SparkMax m_driveMotor;
    private final SparkMaxConfig motorConfigAz;
    private final SparkClosedLoopController m_closedLoopControllerAzimth;
    private final SparkClosedLoopController m_closedLoopControllerDrive;
    public final AbsoluteEncoder m_azimuthEnc; //set in robot container
    public final RelativeEncoder m_driveEnc; // set in robot container, use rev setup, encoder is apart of motorconfig
    private final SparkMaxConfig motorConfigDrive;

    public swervemodule() {
        m_azimuthMotor = new SparkMax(0, null);
        motorConfigAz = new SparkMaxConfig();
        m_closedLoopControllerAzimth = m_azimuthMotor.getClosedLoopController();
        m_azimuthMotor.setSmartCurrentLimit(CurrentLimit.kAzimuth);
        m_azimuthMotor.enableVoltageCompensation(GlobalConstants.kVoltageCompensation);
        m_azimuthMotor.setInverted(false);
        m_azimuthMotor.setIdleMode(IdleMode.kBrake);
        m_azimuthEnc = m_azimuthMotor.getAbsoluteEncoder();
        motorConfigAz.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(.1)
            .i(1)
            .d(1)
            .outputRange(-1, 1)
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
            m_azimuthMotor.configure(motorConfigAz, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            m_driveMotor = new SparkMax(0, null);
            motorConfigDrive = new SparkMaxConfig();
            m_closedLoopControllerDrive = m_driveMotor.getClosedLoopController();
            m_driveMotor.setSmartCurrentLimit(CurrentLimit.kDrive);
            m_driveMotor.enableVoltageCompensation(GlobalConstants.kVoltageCompensation);
            m_driveMotor.setInverted(false);
            m_driveMotor.setIdleMode(IdleMode.kBrake); 
            m_driveEnc = m_driveMotor.getEncoder();
            m_driveEnc.setPositionConversionFactor(Drive.kPositionFactor);
            m_driveEnc.setVelocityConversionFactor(Drive.kVelocityFactor);
            motorConfigDrive.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(.1)
                .i(1)
                .d(1)
                .outputRange(-1, 1)
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
                m_azimuthMotor.configure(motorConfigDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    
}
