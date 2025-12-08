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

    private final SparkMax m_motor = new SparkMax(m_motorPort, MotorType.kBrushless);
        //Creates the shooters motor.

    private final double m_speed = getVelocity();

    private DoubleTopic m_motorTopic = NetworkTableInstance.getDefault().getTable("Shooter")
            .getDoubleTopic("/Shooter/Velocity");
        //Creates location on the smartdaskboard for shooter.
    private DoublePublisher m_motorPublish = m_motorTopic.publish();

    private final SparkMaxConfig m_motorMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private final SparkClosedLoopController m_motorPID;

    private final static int m_motorPort = 3;

    private static double m_motorGearing = Constants.ShooterConstants.kVGearing;

    private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();
    
        //Simulation setup
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_motorEncoderSim = new SparkRelativeEncoderSim(m_motor);
    private final FlywheelSim m_motorFlyWheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 1, m_motorGearing),
                                         DCMotor.getNEO(1).withReduction(m_motorGearing), 
                                         0.0);

    

    public ShooterSubsystem() {
        motorBackground();
    }

     @Override
    public void simulationPeriodic() {
            //Publishes and updates information for simulations.
        m_motorFlyWheelSim.setInputVoltage(m_motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_motorFlyWheelSim.update(0.02);
        m_motorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_motorFlyWheelSim.getAngularVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.2);
        m_motorEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_motorFlyWheelSim.getAngularVelocityRadPerSec()),
                0.2);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_motorFlyWheelSim.getCurrentDrawAmps()));
    }

    private void IntakeStop() {
        m_motorPID.setReference(shooterConstants.kVelocityIntakeStop, ControlType.kVelocity);
    }

    private void ShootStop() {
        m_motorPID.setReference(Units.degreesToRotations((shooterConstants.kVelocityStop)), ControlType.kVelocity);
    }

    @Override
    public void periodic() {       
        m_motorPublish.set(Units.rotationsToDegrees(getVelocity()));
            SmartDashboard.getNumber("shooter Velocity", getVelocity());
    }

    public double getVelocity() {
        double Velocity = m_motorEncoder.getVelocity();
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
        m_motorPID.setReference(m_speed, ControlType.kVelocity);
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();

        m_motorMotorConfig.encoder
          .positionConversionFactor(m_motorGearing)
          .velocityConversionFactor(m_motorGearing / 60);

        m_motorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.ShooterConstants.kVP)
                .i(Constants.ShooterConstants.kVI)
                .d(Constants.ShooterConstants.kVD)
                .outputRange(-1, 1);
        m_motorMotorConfig.idleMode(IdleMode.kBrake);
        m_motor.configure(m_motorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}