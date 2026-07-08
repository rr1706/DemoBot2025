package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class Intake extends SubsystemBase {
    private final SparkMax m_roller = 
            new SparkMax(intakeConstants.kRollerMotorPort, MotorType.kBrushless);
    private final SparkMax m_arm = 
            new SparkMax(intakeConstants.kArmMotorPort, MotorType.kBrushless);

    private final RelativeEncoder m_rollerEncoder = m_roller.getEncoder();
    private final RelativeEncoder m_armEncoder = m_arm.getEncoder();

    private final SparkMaxConfig m_rollerConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_armConfig = new SparkMaxConfig();

    private SparkClosedLoopController m_rollerPID = m_roller.getClosedLoopController();
    private SparkClosedLoopController m_armPID = m_arm.getClosedLoopController();

    private static double m_setVelocity = 0.0;
    private static double m_setPosition = 0.0;

    public Intake() {
        motorConfigs();
    }

    public double getVelocity() {
        return m_rollerEncoder.getVelocity();
    }

    public double getPosition() {
        return m_armEncoder.getPosition();
    }

    public void setVelocity(double Velocity) {
        m_setVelocity = Velocity;
        m_rollerPID.setReference(m_setVelocity, ControlType.kMAXMotionVelocityControl);
    }

    public void setPosition(double Position) {
        m_setPosition = Position;
        m_armPID.setReference(m_setPosition, ControlType.kMAXMotionPositionControl);
    }

    public void intake() {
        setVelocity(intakeConstants.kIntakeVelocity);
        setPosition(intakeConstants.kIntakePosition);
    }

    public void home() {
        setVelocity(0.0);
        setPosition(intakeConstants.kHomePosition);
    }

    public void stop() {
        m_roller.stopMotor();
        m_arm.stopMotor();
    }

    public Command intakeCmd() {
        return runEnd(()-> intake(), ()-> home());
    }

    private void motorConfigs() {
        m_rollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(intakeConstants.kP)
                .i(intakeConstants.kI)
                .d(intakeConstants.kD)
                .outputRange(-1, 1);

        m_armConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(intakeConstants.kArmP)
                .i(intakeConstants.kArmI)
                .d(intakeConstants.kArmD)
                .outputRange(-1, 1);

        m_armConfig.softLimit
                .forwardSoftLimit(intakeConstants.kForwardLimit)
                .reverseSoftLimit(intakeConstants.kReverseLimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        
        m_rollerConfig.closedLoop.maxMotion
                .maxAcceleration(intakeConstants.kMAXACCEL)
                .maxVelocity(intakeConstants.kMAXVELOCITY)
                .allowedClosedLoopError(1.0);

        m_rollerConfig.closedLoop.maxMotion
                .maxAcceleration(intakeConstants.kARM_MAXACCEL)
                .maxVelocity(intakeConstants.kARM_MAXVELOCITY)
                .allowedClosedLoopError(1.0);

        m_rollerConfig.idleMode(IdleMode.kBrake);
        m_rollerConfig.smartCurrentLimit(intakeConstants.kRollerLimit);

        m_armConfig.idleMode(IdleMode.kBrake);
        m_armConfig.smartCurrentLimit(intakeConstants.kArmLimit);

        m_roller.configure(m_rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_arm.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Set Velocity", m_setVelocity);
        SmartDashboard.putNumber("Intake Velocity", getVelocity());
        SmartDashboard.putNumber("Intake Set Position", m_setPosition);
        SmartDashboard.putNumber("Intake Position", getPosition());
    }
}
