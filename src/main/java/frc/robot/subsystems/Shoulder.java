package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;


public class Shoulder extends SubsystemBase {
    private final SparkMax m_motor = 
            new SparkMax(Constants.ShoulderConstants.kMotorPort, MotorType.kBrushless);
    private final SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_motorPID;
    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();

    public Shoulder() {
        motorBackground();
    }

    private double m_setAngle = Constants.ShoulderConstants.kDefault;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pitcher Set Angle", m_setAngle);
        SmartDashboard.putNumber("Pitcher True Angle", getPosition());
    }

    public double getSetAngle() {
        return m_setAngle;
    }

    public double getPosition() {
        double Position = m_motorEncoder.getPosition();
        return Units.radiansToDegrees(Position);
    }

    public double getVelocity() {
        double Velocity = m_motorEncoder.getVelocity();
        return (Velocity);
    }

    public double changePitch(double adjust) {
        m_setAngle += adjust;
        SmartDashboard.putNumber("Changed Pitcher Angle", adjust);
        return m_setAngle;
    }

    public void Intake(){
        m_setAngle = Constants.ShoulderConstants.kIntake;
        m_motorPID.setReference(m_setAngle, ControlType.kMAXMotionPositionControl);
    }

    public void Shoot() {
        m_setAngle = Constants.ShoulderConstants.kShoot;
        m_motorPID.setReference(m_setAngle, ControlType.kMAXMotionPositionControl);
    }

    public void Home() {
        m_setAngle = Constants.ShoulderConstants.kDefault;
        m_motorPID.setReference(m_setAngle, ControlType.kMAXMotionPositionControl);
    }

    public void setAngle(double angle) {
        m_setAngle = angle;

        if (m_setAngle >= Constants.ShoulderConstants.kMax) {
            m_setAngle = Constants.ShoulderConstants.kMax;
        } else if (m_setAngle <= Constants.ShoulderConstants.kMin) {
            m_setAngle = Constants.ShoulderConstants.kMin;
        }

        m_motorPID.setReference(m_setAngle, ControlType.kMAXMotionPositionControl);
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();

        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ShoulderConstants.kP)
                .i(Constants.ShoulderConstants.kI)
                .d(Constants.ShoulderConstants.kD)
                .outputRange(-1, 1);

        m_motorConfig.closedLoop.maxMotion
                .maxAcceleration(Constants.ShoulderConstants.maxMotion.kA)
                .maxVelocity(Constants.ShoulderConstants.maxMotion.kV)
                .allowedClosedLoopError(Constants.ShoulderConstants.maxMotion.kE);
        

        m_motorConfig.softLimit
                .forwardSoftLimit(ShoulderConstants.kforwardLim)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ShoulderConstants.kreverseLim)
                .reverseSoftLimitEnabled(true);

        m_motorConfig.idleMode(IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(Constants.CurrentLimit.kShoulder);
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}