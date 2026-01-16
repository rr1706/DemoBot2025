package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class Shooter extends SubsystemBase {

    private final SparkMax m_motor = 
            new SparkMax(Constants.shooterConstants.kMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();
    private final SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_motorPID;
    private final Debouncer m_debounce = new Debouncer(0.090, DebounceType.kBoth);

    public Shooter() {
        motorBackground();
    }

    private double m_setSpeed = 0.0;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Set Velocity", m_setSpeed);
        SmartDashboard.putNumber("Shooter True Velocity", getVelocity());
    }

    public boolean hasBall(){
        return m_debounce.calculate(m_motor.getOutputCurrent() >= 80.0 && m_setSpeed < -0.1);
    }

    public void Intake() {
        m_setSpeed = shooterConstants.kIntake;
        m_motorPID.setReference(m_setSpeed, ControlType.kMAXMotionVelocityControl);
    }

    public void Shoot() {
        m_setSpeed = shooterConstants.kShoot;
        m_motorPID.setReference(m_setSpeed, ControlType.kMAXMotionVelocityControl);
    }

    public void Stop() {
        m_setSpeed = shooterConstants.kDefault;
        m_motorPID.setReference(m_setSpeed, ControlType.kMAXMotionVelocityControl);
    }

    public double getVelocity() {
        double Velocity = m_motorEncoder.getVelocity();
        return (Velocity);
    }

    public double changeVelocity(double adjust) {
        m_setSpeed += adjust;
        SmartDashboard.putNumber("Changed Shooter Velocity", adjust);
        return m_setSpeed;
    }

    public void setVelocity(double speed) {
        m_setSpeed = speed;
        m_motorPID.setReference(m_setSpeed, ControlType.kMAXMotionVelocityControl);
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();

        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(shooterConstants.kP)
                .i(shooterConstants.kI)
                .velocityFF(shooterConstants.kFF)
                .outputRange(-1, 1);
        
        m_motorConfig.closedLoop.maxMotion
                .maxAcceleration(shooterConstants.kMAXACCEL)
                .maxVelocity(shooterConstants.kMAXVELOCITY)
                .allowedClosedLoopError(1);

        m_motorConfig.idleMode(IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(Constants.shooterConstants.kLimit);
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}