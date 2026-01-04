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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shoulder extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.ShoulderConstants.kMotorPort, MotorType.kBrushless);
        //Creates the pitchers motor.

    private final SparkMaxConfig m_motorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private SparkClosedLoopController m_motorPID;

    private static double m_motorGearing = 25;

    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();

    public Shoulder() {
        motorBackground();
    }

    private double m_setAngle = Constants.ShoulderConstants.kDefault;
    private double m_setPower = Constants.ShoulderConstants.kPDefault;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pitcher Set Angle", m_setAngle);
        SmartDashboard.putNumber("Pitcher Set Speed", m_setPower);
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
        m_motorPID.setReference(m_setPower, ControlType.kVelocity);
        m_motorPID.setReference(Units.degreesToRadians(m_setAngle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void Shoot() {
        m_setAngle = Constants.ShoulderConstants.kShoot;
        m_motorPID.setReference(m_setPower, ControlType.kVelocity);
        m_motorPID.setReference(Units.degreesToRadians(m_setAngle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void Home() {
        m_setAngle = Constants.ShoulderConstants.kDefault;
        m_motorPID.setReference(m_setPower, ControlType.kVelocity);
        m_motorPID.setReference(Units.degreesToRadians(m_setAngle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setAngle(double angle, double power) {
        m_setAngle = angle;
        m_setPower = power;

        if (m_setAngle >= Constants.ShoulderConstants.kMax) {
            m_setAngle = Constants.ShoulderConstants.kMax;
        } else if (m_setAngle <= Constants.ShoulderConstants.kMin) {
            m_setAngle = Constants.ShoulderConstants.kMin;
        }

        if (m_setPower >= Constants.ShoulderConstants.kPMax) {
            m_setPower = Constants.ShoulderConstants.kPMax;
        } else if (m_setPower <= Constants.ShoulderConstants.kPMin) {
            m_setPower = Constants.ShoulderConstants.kPMin;
        } else if (m_setPower == 0) {
            m_setPower = Constants.ShoulderConstants.kPDefault;
        }

        m_motorPID.setReference(m_setPower, ControlType.kVelocity);
        m_motorPID.setReference(Units.degreesToRadians(m_setAngle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();
        
        m_motorConfig.encoder
            .positionConversionFactor(m_motorGearing)
            .velocityConversionFactor(m_motorGearing/60);

        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ShoulderConstants.kP)
                .i(Constants.ShoulderConstants.kI)
                .d(Constants.ShoulderConstants.kD)
                .outputRange(-1, 1);

        m_motorConfig.idleMode(IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(Constants.CurrentLimit.kShoulder);
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}