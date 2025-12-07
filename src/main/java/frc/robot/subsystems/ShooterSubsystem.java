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

    private final double m_speed = getVelocity();

    private DoubleTopic m_shooterTopic = NetworkTableInstance.getDefault().getTable("Shooter")
            .getDoubleTopic("/Shooter/Velocity");
        //Creates location on the smartdaskboard for shooter.
    private DoublePublisher m_shooterPublish = m_shooterTopic.publish();

    private final SparkMaxConfig m_shooterMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private final SparkClosedLoopController m_shooterPID;

    private final static int m_shooterPort = 3;

    private static double m_shooterGearing = Constants.ShooterConstants.kVGearing;

    private final RelativeEncoder m_shooterEncoder = m_Shooter.getEncoder();
    
        //Simulation setup
    private final SparkMaxSim m_shooterSim = new SparkMaxSim(m_Shooter, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_shooterEncoderSim = new SparkRelativeEncoderSim(m_Shooter);
    private final FlywheelSim m_shooterFlyWheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1, m_shooterGearing),
                                         DCMotor.getNEO(1).withReduction(m_shooterGearing), 
                                         0.0);

    

    public ShooterSubsystem() {

        m_shooterPID = m_Shooter.getClosedLoopController();

        m_shooterMotorConfig.encoder
          .positionConversionFactor(m_shooterGearing)
          .velocityConversionFactor(m_shooterGearing / 60);

        m_shooterMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ShooterConstants.kVP)
                .i(Constants.ShooterConstants.kVI)
                .d(Constants.ShooterConstants.kVD)
                .outputRange(-1, 1);
        m_shooterMotorConfig.idleMode(IdleMode.kBrake);
        m_Shooter.configure(m_shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
    }

        private void IntakeStop() {
            m_shooterPID.setReference(shooterConstants.kVelocityIntakeStop, ControlType.kVelocity);
        }

        private void ShootStop() {
            m_shooterPID.setReference(Units.degreesToRotations((shooterConstants.kVelocityStop)), ControlType.kVelocity);
        }

    @Override
    public void periodic() {       
        m_shooterPublish.set(Units.rotationsToDegrees(getVelocity()));
            SmartDashboard.getNumber("shooter Velocity", getVelocity());
    }

    public double getVelocity() {
        double Velocity = m_shooterEncoder.getVelocity();
        SmartDashboard.getNumber("Shooter Velocity", Velocity);
        return Velocity;
    }

    public void changeVelocity(double adjust) {
        return this.run(() -> m_speed += adjust);
    }

    public Command IntakeCommandStop() {
        return this.run(() -> this.IntakeStop());
    }

    public Command ShootCommandStop() {
        return this.run(() -> this.ShootStop());
    }

    public void setVelocity(double speed) {
        m_speed = speed;
        m_ShooterPID.setReference(m_speed, ControlType.kVelocity);
    }
}