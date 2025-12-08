package frc.robot.subsystems

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class AngleSubsystem extends SubsystemBase {

    private final SparkMax m_motor = new SparkMax(m_motorPort, MotorType.kBrushless);
        //Creates the pitchers motor.
    
    private double m_angle = Constants.shooterConstants.kangle;
        //Creates the angle var thats used for setting angle and adjust.

    private DoubleTopic m_motorTopic = NetworkTableInstance.getDefault().getTable("Shooter Angle")
            .getDoubleTopic("/Shooter/Angle");
        //Creates location on the smartdashboard for the pitcher.
    private DoublePublisher m_motorPublish = m_motorTopic.publish();

    private final SparkMaxConfig m_motorMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private final SparkClosedLoopController m_motorPID;

    private final static int m_motorPort = 4;

    private static double m_motorGearing = Constants.ShooterConstants.kAGearing;

    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();

        //Simulations setup
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_motorEncoderSim = new SparkRelativeEncoderSim(m_motor);
    private final SingleJointedArmSim m_motorArmSim = new SingleJointedArmSim(DCMotor.getNEO(1),
            m_motorGearing,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), Units.lbsToKilograms(4)),
            Units.inchesToMeters(12),
            0,
            Math.PI / 2,
            true,
            Math.PI / 2,
            0.0, 0.0);
    
    public AngleSubsystem() {
        motorBackground();
    }

    @Override
    public void simulationPeriodic() {
        m_motorArmSim.setInputVoltage(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_motorArmSim.update(0.02);
        m_motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_motorArmSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.2);
        m_motorEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_motorArmSim.getVelocityRadPerSec()),
                0.2);
    }

     @Override
    public void periodic() {
        m_motorPublish.set(Units.rotationsToDegrees(getPosition()));
                // Converts to correct unit then published to SmartDashboard.
                SmartDashboard.putNumber("setAngle", m_angle);
        m_motorPID.setReference(m_angle, ControlType.kPosition);
    }
    public double getPosition() {
        double Position = m_motorEncoder.getPosition();
        SmartDashboard.getNumber("Shooter Position", Position);
        return Units.rotationsToDegrees(Position);
    }

    public double getSetAngle() {
        double setPose = m_angle;
        return setPose;
    }

    public Command changePitch(double adjust) {
        return runOnce(() -> {m_angle += adjust;});
    }

    public void setAngle(double angle) {
        m_angle = angle;
        if (angle >= Constants.ShooterConstants.kAMax) {
            angle = Constants.ShooterConstants.kAMax;
        }
        m_motorPID.setReference(Units.degressToRotations(angle), ControlType.kPosition);
    }

    public Command AngleHardStop(){
        return this.runOnce(() ->m_motor.stopMotor());
    }

    public Command ShootAngleCommand() {
        return this.run(() ->  setAngle(5.0));
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();
        
        m_motorMotorConfig.encoder
            .positionConversionFactor(m_motorGearing)
            .velocityConversionFactor(m_motorGearing/60);

        m_motorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ShooterConstants.kAP)
                .i(Constants.ShooterConstants.kAI)
                .d(Constants.ShooterConstants.kAD)
                .outputRange(-1, 1);
        m_motorMotorConfig.idleMode(IdleMode.kBrake);
        m_motor.configure(m_motorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
