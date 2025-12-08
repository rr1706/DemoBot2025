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

    private final SparkMax m_ShooterAngle = new SparkMax(m_shooterAnglePort, MotorType.kBrushless);
        //Creates the pitchers motor.
    
    private double m_angle = Constants.shooterConstants.kangle;
        //Creates the angle var thats used for setting angle and adjust.

    private DoubleTopic m_shooterAngleTopic = NetworkTableInstance.getDefault().getTable("Shooter Angle")
            .getDoubleTopic("/Shooter/Angle");
        //Creates location on the smartdashboard for the pitcher.
    private DoublePublisher m_shooterAnglePublish = m_shooterAngleTopic.publish();

    private final SparkMaxConfig m_shooterAngleMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private final SparkClosedLoopController m_shooterAnglePID;

    private final static int m_shooterAnglePort = 4;

    private static double m_shooterAngleGearing = Constants.ShooterConstants.kAGearing;

    private final RelativeEncoder m_shooterAngleEncoder = m_ShooterAngle.getEncoder();

        //Simulations setup
    private final SparkMaxSim m_shooterAngleSim = new SparkMaxSim(m_ShooterAngle, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_ShooterEncoderSim = new SparkRelativeEncoderSim(m_ShooterAngle);
    private final SingleJointedArmSim m_ShooterArmSim = new SingleJointedArmSim(DCMotor.getNEO(1),
            m_shooterAngleGearing,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), Units.lbsToKilograms(4)),
            Units.inchesToMeters(12),
            0,
            Math.PI / 2,
            true,
            Math.PI / 2,
            0.0, 0.0);
    
    public AngleSubsystem() {
        m_shooterAnglePID = m_ShooterAngle.getClosedLoopController();
        
        m_shooterAngleMotorConfig.encoder
            .positionConversionFactor(m_shooterAngleGearing)
            .velocityConversionFactor(m_shooterAngleGearing/60);

        m_shooterAngleMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ShooterConstants.kAP)
                .i(Constants.ShooterConstants.kAI)
                .d(Constants.ShooterConstants.kAD)
                .outputRange(-1, 1);
        m_shooterAngleMotorConfig.idleMode(IdleMode.kBrake);
        m_ShooterAngle.configure(m_shooterAngleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void simulationPeriodic() {
        m_ShooterArmSim.setInputVoltage(m_shooterAngleSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_ShooterArmSim.update(0.02);
        m_shooterAngleSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_ShooterArmSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.2);
        m_ShooterEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_ShooterArmSim.getVelocityRadPerSec()),
                0.2);
    }

     @Override
    public void periodic() {
        m_shooterAnglePublish.set(Units.rotationsToDegrees(getPosition()));
                // Converts to correct unit then published to SmartDashboard.
                SmartDashboard.putNumber("setAngle", m_angle);
        m_shooterAnglePID.setReference(m_angle, ControlType.kPosition);

    public double getPosition() {
        double Position = m_shooterAngleEncoder.getPosition();
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
        m_shooterAnglePID.setReference(Units.degressToRotations(angle), ControlType.kPosition);
    }

    public Command AngleHardStop(){
        return this.runOnce(() ->m_shooterAngle.stopMotor());
    }

    public Command ShootAngleCommand() {
        return this.run(() ->  setAngle(5.0));
    }
}
}