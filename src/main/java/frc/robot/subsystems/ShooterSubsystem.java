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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax m_Shooter = new SparkMax(m_shooterPort, MotorType.kBrushless);
        //Creates the shooters motor.

    private final SparkMax m_ShooterAngle = new SparkMax(m_shooterAnglePort, MotorType.kBrushless);
        //Creates the pitchers motor.

    private double m_angle = Constants.shooterConstants.kangle;
        //Creates the angle var thats used for setting angle and adjust.

    private DoubleTopic m_shooterTopic = NetworkTableInstance.getDefault().getTable("Shooter")
            .getDoubleTopic("/Shooter/Velocity");
        //Creates location on the smartdaskboard for shooter.
    private DoublePublisher m_shooterPublish = m_shooterTopic.publish();

    private DoubleTopic m_shooterAngleTopic = NetworkTableInstance.getDefault().getTable("Shooter Angle")
            .getDoubleTopic("/Shooter/Angle");
        //Creates location on the smartdashboard for the pitcher.
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

    
        //Simulation setup (shooter)
    private final SparkMaxSim m_shooterSim = new SparkMaxSim(m_Shooter, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_shooterEncoderSim = new SparkRelativeEncoderSim(m_Shooter);
    private final FlywheelSim m_shooterFlyWheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1, m_shooterGearing),
                                         DCMotor.getNEO(1).withReduction(m_shooterGearing), 
                                         0.0);

        //Simulations setup (pitcher)
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

            //Publishes and updates information for simulations.
        m_shooterFlyWheelSim.setInputVoltage(m_shooterSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_shooterFlyWheelSim.update(0.02);
        m_shooterSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_shooterFlyWheelSim.getAngularVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.2);
        m_shooterEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_shooterFlyWheelSim.getAngularVelocityRadPerSec()),
                0.2);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shooterFlyWheelSim.getCurrentDrawAmps()));

        m_ShooterArmSim.setInputVoltage(m_shooterAngleSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_ShooterArmSim.update(0.02);
        m_shooterAngleSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_ShooterArmSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.2);
        m_ShooterEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_ShooterArmSim.getVelocityRadPerSec()),
                0.2);
    }

        private void Shoot() {
            m_shooterPID.setReference(shooterConstants.kVelocity, ControlType.kVelocity);
        }

        private void Intake() {
            m_shooterPID.setReference(shooterConstants.kVelocityIntake, ControlType.kVelocity);
        }

        private void IntakeStop() {
            m_shooterPID.setReference(shooterConstants.kVelocityIntakeStop, ControlType.kVelocity);
        }

        private void ShootStop() {
            m_shooterPID.setReference(Units.degreesToRotations((shooterConstants.kVelocityStop)), ControlType.kVelocity);
        }

    @Override
    public void periodic() {
        m_shooterAnglePublish.set(Units.rotationsToDegrees(getPosition()));
                // Converts to correct unit then published to SmartDashboard.
                SmartDashboard.putNumber("setAngle", m_angle);


        m_shooterPublish.set(Units.rotationsToDegrees(getVelocity()));
        SmartDashboard.getNumber("shooter Angle", getPosition());
        m_shooterAnglePID.setReference(m_angle, ControlType.kPosition);


    }

    public double getVelocity() {
        double Velocity = m_shooterEncoder.getVelocity();
        SmartDashboard.getNumber("Shooter Velocity", Velocity);
        return Velocity;
    }

    public double getPosition() {
        double Position = m_shooterAngleEncoder.getPosition();
        SmartDashboard.getNumber("Shooter Position", Position);
        return Units.rotationsToDegrees(Position);
    }

    public Command ShootCommand() {
        return this.run(() -> this.Shoot());
    }
    
    public Command IntakeCommand() {
        return this.run(() -> this.Intake());
    }

    public Command IntakeCommandStop() {
        return this.run(() -> this.IntakeStop());
    }

    public Command ShootCommandStop() {
        return this.run(() -> this.ShootStop());
    }

    public Command changePitch(double adjust) {
        return runOnce(() -> {
            m_angle += adjust;
        });
    }

    public void setAngle(double angle) {
        m_shooterAnglePID.setReference(Units.degressToRotations(angle), ControlType.kPosition);
    }

    public Command ShootAngleCommand() {
    return this.run(() ->  setAngle(5.0));
    }
}

