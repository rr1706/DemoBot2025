package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private SparkMax m_Shooter = new SparkMax(m_shooterPort, MotorType.kBrushless);
    private SparkMax m_ShooterAngle = new SparkMax(m_shooterAnglePort, MotorType.kBrushless);

    private DoubleTopic m_shooterTopic = NetworkTableInstance.getDefault().getTable("Shooter")
            .getDoubleTopic("/Shooter/Velocity");
    private DoublePublisher m_shooterPublish = m_shooterTopic.publish();
    private DoubleTopic m_shooterAngleTopic = NetworkTableInstance.getDefault().getTable("Shooter Angle")
            .getDoubleTopic("/Shooter/Angle");
    private DoublePublisher m_shooterAnglePublish = m_shooterAngleTopic.publish();

    private final SparkMaxConfig m_shooterMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.
    private final SparkMaxConfig m_shooterAngleMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.


    private final SparkClosedLoopController m_shooterPID;
    private final SparkClosedLoopController m_shooterAnglePID;

    private final static int m_shooterPort = 3;
    private final static int m_shooterAnglePort = 4;

    private static double m_shooterGearing = 1;
    private static double m_shooterAngleGearing = 1;

    private final RelativeEncoder m_shooterEncoder = m_Shooter.getEncoder();
    private final RelativeEncoder m_shooterAngleEncoder = m_ShooterAngle.getEncoder();

    
    private final SparkMaxSim m_shooterSim = new SparkMaxSim(m_Shooter, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_shooterEncoderSim = new SparkRelativeEncoderSim(m_Shooter);
    private final FlywheelSim m_shooterFlyWheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1, m_shooterGearing),
                                         DCMotor.getNEO(1).withReduction(m_shooterGearing), 
                                         0.0);
    

    public ShooterSubsystem() {

        m_shooterPID = m_Shooter.getClosedLoopController();
        m_shooterAnglePID = m_ShooterAngle.getClosedLoopController();

        m_shooterMotorConfig.encoder
          .positionConversionFactor(m_shooterGearing)
          .velocityConversionFactor(m_shooterGearing / 60);

        m_shooterAngleMotorConfig.encoder
            .positionConversionFactor(m_shooterAngleGearing)
            .velocityConversionFactor(m_shooterAngleGearing/60);

        m_shooterMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1);
        m_shooterMotorConfig.idleMode(IdleMode.kBrake);
        m_Shooter.configure(m_shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_shooterAngleMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1);
        m_shooterAngleMotorConfig.idleMode(IdleMode.kBrake);
        m_ShooterAngle.configure(m_shooterAngleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

     @Override
    public void simulationPeriodic() {
        m_shooterFlyWheelSim.setInputVoltage(m_shooterSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_shooterFlyWheelSim.update(0.02);
        m_shooterSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_shooterFlyWheelSim.getAngularVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.2);
        m_shooterEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_shooterFlyWheelSim.getAngularVelocityRadPerSec()),
                0.2);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shooterFlyWheelSim.getCurrentDrawAmps()));
    }


        private void Shoot() {
            m_shooterPID.setReference(Units.degreesToRotations(ShooterConstants.kVelocity), ControlType.kVelocity);
        }

        private void ShootAngle() {
            m_shooterAnglePID.setReference(Units.degreesToRotations(ShooterConstants.kShooterAngleUp), ControlType.kPosition);
        }

        private void ShootStop() {
            m_shooterPID.setReference(Units.degreesToRotations(ShooterConstants.kVelocityStop), ControlType.kVelocity);
        }

        private void ShootAngleDown() {
            m_shooterAnglePID.setReference(Units.degreesToRotations(ShooterConstants.kShooterAngleDown), ControlType.kPosition);
        }

    @Override
    public void periodic() {
        m_shooterAnglePublish.set(Units.rotationsToDegrees(getPosition()));
                // Converts to correct unit then published to SmartDashboard.

        m_shooterPublish.set(Units.rotationsToDegrees(getVelocity()));
    }

    public double getVelocity() {
        double Velocity = m_shooterEncoder.getVelocity();
        SmartDashboard.getNumber("Shooter Velocity", Velocity);
        return Velocity;
    }

    public double getPosition() {
        double Position = m_shooterAngleEncoder.getPosition();
        SmartDashboard.getNumber("shooter Angle", Position);
        return Position;
    }

    public Command ShootCommand2() {
        return this.runEnd(() -> {
                
        {m_shooterPID.setReference(Units.degreesToRotations(ShooterConstants.kVelocity), ControlType.kVelocity);}
                    
        }, () -> {m_shooterPID.setReference(Units.degreesToRotations(ShooterConstants.kVelocityStop), ControlType.kVelocity);});
            
        }

    public Command ShootCommand() {
        return this.run(() -> this.Shoot());
    }
        
    public Command ShootCommandStop() {
        return this.run(() -> this.ShootStop());
    }

    public Command ShooterAngleAjustUpCommand() {
        return this.run(() -> this.ShootAngle());
    }

    public Command ShooterAngleAjustDownCommand() {
        return this.run(() -> this.ShootAngleDown());
    }
}

