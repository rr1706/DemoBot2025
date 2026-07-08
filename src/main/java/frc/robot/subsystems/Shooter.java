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
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {
    private final SparkMax m_motor = 
            new SparkMax(shooterConstants.kMotorPort, MotorType.kBrushless);

    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();
    private final SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_motorPID;

    public Shooter() {
        motorConfigs();
    }

    private double m_setpoint = 0.0;

    public double getVelocity() {
        return m_motorEncoder.getVelocity();
    }

    public void setVelocity(double speed) {
        m_setpoint = speed;
        m_motorPID.setReference(m_setpoint, ControlType.kMAXMotionVelocityControl);
    }

    public void shoot() {
        setVelocity(shooterConstants.kshoot);
    }

    public void stop() {
        m_setpoint = 0.0;
        m_motor.stopMotor();
    }

    public double changeVelocity(double adjust) {
        m_setpoint += adjust;
        SmartDashboard.putNumber("Changed Shooter Velocity", adjust);
        return m_setpoint;
    }

    public Command shootCmd() {
        return runEnd(()-> shoot(), ()-> stop());
    }

    private void motorConfigs() {
        m_motorPID = m_motor.getClosedLoopController();

        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(shooterConstants.kP)
                .i(shooterConstants.kI)
                .d(shooterConstants.kD)
                .velocityFF(shooterConstants.kFF)
                .outputRange(-1, 1);
        
        m_motorConfig.closedLoop.maxMotion
                .maxAcceleration(shooterConstants.kMAXACCEL)
                .maxVelocity(shooterConstants.kMAXVELOCITY)
                .allowedClosedLoopError(1);

        m_motorConfig.idleMode(IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(shooterConstants.kLimit);
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Set Velocity", m_setpoint);
        SmartDashboard.putNumber("Shooter True Velocity", getVelocity());
    }
}
