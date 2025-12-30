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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AngleSubsystem extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(m_motorPort, MotorType.kBrushless);
        //Creates the pitchers motor.

    private final SparkMaxConfig m_motorMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private SparkClosedLoopController m_motorPID;

    private final static int m_motorPort = 10;

    private static double m_motorGearing = 25;

    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();

    public AngleSubsystem() {
        motorBackground();
    }

    private double m_angle = getPosition();
    private double m_setAngle;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle", m_angle);
        SmartDashboard.putNumber("Shooter Set Angle", m_setAngle);
                // Converts to correct unit then published to SmartDashboard.
    }

    public double getPosition() {
        double Position = m_motorEncoder.getPosition();
        return Units.rotationsToDegrees(Position);
    }

    public double getVelocity() {
        double Velocity = m_motorEncoder.getVelocity();
        return Units.rotationsToDegrees(Velocity);
    }

    public double getSetAngle() {
        return m_setAngle;
    }

    public double changePitch(double adjust) {
        m_setAngle += adjust;
        return m_setAngle;
    }

    private void Intake(){
        m_setAngle = Constants.shooterConstants.kAngleIntake;
        m_motorPID.setReference(Units.degreesToRadians(Constants.shooterConstants.kAngleIntake), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private void Shoot() {
        m_setAngle = Constants.shooterConstants.kAngleShoot;
        m_motorPID.setReference(Units.degreesToRadians(Constants.shooterConstants.kAngleShoot), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private void Home() {
        m_setAngle = Constants.shooterConstants.kAngleStop;
        m_motorPID.setReference(Units.degreesToRadians(Constants.shooterConstants.kAngleStop), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setAngle(double angle) {
        m_setAngle = angle;
        if (m_setAngle >= Constants.shooterConstants.kAMax) {
            m_setAngle = Constants.shooterConstants.kAMax;
        } else if (m_setAngle <= Constants.shooterConstants.kAMin) {
            m_setAngle = Constants.shooterConstants.kAMin;
        }
        m_motorPID.setReference(Units.degreesToRadians(m_setAngle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Command AngleHardStop(){
        return runOnce(() ->m_motor.stopMotor());
    }

    public Command ShootCommand() {
        return this.run(() -> this.Shoot());
    }

    public Command IntakeCommand() {
        return this.run(() -> this.Intake());
    }

    public Command HomeCommand() {
        return this.run(() -> this.Home());
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();
        
        m_motorMotorConfig.encoder
            .positionConversionFactor(m_motorGearing)
            .velocityConversionFactor(m_motorGearing/60);

        m_motorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.shooterConstants.kAP)
                .i(Constants.shooterConstants.kAI)
                .d(Constants.shooterConstants.kAD)
                .outputRange(-1, 1);
        m_motorMotorConfig.idleMode(IdleMode.kBrake);
        m_motor.configure(m_motorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}