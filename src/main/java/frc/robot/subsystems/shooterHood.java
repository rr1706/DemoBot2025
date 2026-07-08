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
import frc.robot.Constants.shooterHoodConstants;


public class ShooterHood extends SubsystemBase {
    private final SparkMax m_motor = 
            new SparkMax(shooterHoodConstants.kMotorPort, MotorType.kBrushless);

    private final SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_motorPID;
    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();

    public ShooterHood() {
        motorConfigs();
    }

    private static double m_setpoint = 0.0;

    public double getPosition() {
        return m_motorEncoder.getPosition();
    }

    public void setAngle(double angle) {
        m_setpoint = angle;

        if (m_setpoint >= shooterHoodConstants.kMax) {
            m_setpoint = shooterHoodConstants.kMax;
        } else if (m_setpoint <= shooterHoodConstants.kMin) {
            m_setpoint = shooterHoodConstants.kMin;
        }

        m_motorPID.setReference(m_setpoint, ControlType.kMAXMotionPositionControl);
    }

    public double changePitch(double adjust) {
        m_setpoint += adjust;
        SmartDashboard.putNumber("Changed Pitcher Angle", adjust);
        return m_setpoint;
    }

    public void shoot() {
        setAngle(shooterHoodConstants.kShoot);
    }

    public void home() {
        setAngle(shooterHoodConstants.kDefault);
    }

    public Command shootCmd() {
        return runEnd(()-> shoot(), ()-> home());
    }

    private void motorConfigs() {
        m_motorPID = m_motor.getClosedLoopController();

        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(shooterHoodConstants.kP)
                .i(shooterHoodConstants.kI)
                .d(shooterHoodConstants.kD)
                .outputRange(-1, 1);

        m_motorConfig.closedLoop.maxMotion
                .maxAcceleration(shooterHoodConstants.kMAXACCEL)
                .maxVelocity(shooterHoodConstants.kMAXVELOCITY)
                .allowedClosedLoopError(1.0);
        

        m_motorConfig.softLimit
                .forwardSoftLimit(shooterHoodConstants.kForwardLim)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(shooterHoodConstants.kReverseLim)
                .reverseSoftLimitEnabled(true);

        m_motorConfig.idleMode(IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(shooterHoodConstants.kLimit);
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pitcher Set Angle", m_setpoint);
        SmartDashboard.putNumber("Pitcher True Angle", getPosition());
    }
}