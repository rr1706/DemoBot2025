package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax m_motor = new SparkMax(m_motorPort, MotorType.kBrushless);
        //Creates the shooters motor.
        private final RelativeEncoder m_motorEncoder = m_motor.getEncoder();
    
        private DoubleTopic m_motorTopic = NetworkTableInstance.getDefault().getTable("Shooter")
                .getDoubleTopic("/Shooter/Velocity");
            //Creates location on the smartdaskboard for shooter.
        private DoublePublisher m_motorPublish = m_motorTopic.publish();
    
        private final SparkMaxConfig m_motorMotorConfig = new SparkMaxConfig();
        // Creates the motors configurations.
    
        private SparkClosedLoopController m_motorPID;
    
        private final static int m_motorPort = 9;
    
        private static double m_motorGearing = 1;
    
        public ShooterSubsystem() {
            motorBackground();
        }

        private double m_speed = getVelocity();
        private double m_setSpeed;
    
        private void Intake() {
            m_setSpeed = shooterConstants.kVelocityIntake;
            m_motorPID.setReference(shooterConstants.kVelocityIntake, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    
        private void Shoot() {
            m_setSpeed = shooterConstants.kVelocityShoot;
            m_motorPID.setReference(shooterConstants.kVelocityShoot, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }

        private void Stop() {
            m_setSpeed = shooterConstants.kVelocityStop;
            m_motorPID.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    
        @Override
        public void periodic() {       
            m_motorPublish.set(m_setSpeed);
            SmartDashboard.putNumber("shooter Velocity", getVelocity());
        }
    
        public double getVelocity() {
            double Velocity = m_motorEncoder.getVelocity();
            SmartDashboard.putNumber("Shooter Velocity", Velocity);
            return Velocity;
        }
    
        public double changeVelocity(double adjust) {
            m_setSpeed += adjust;
            return m_setSpeed;
        }

    public Command ShootCommand() {
        return this.run(() -> this.Shoot());
    }

    public Command IntakeCommand() {
        return this.run(() -> this.Intake());
    }

    public Command StopCommand() {
        return this.run(() -> this.Stop());
    }

    public void setVelocity(double speed) {
        m_setSpeed = speed;
        m_motorPID.setReference(m_setSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    private void motorBackground() {
        m_motorPID = m_motor.getClosedLoopController();

        m_motorMotorConfig.encoder
          .positionConversionFactor(m_motorGearing)
          .velocityConversionFactor(m_motorGearing / 60);

        m_motorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.shooterConstants.kVP)
                .i(Constants.shooterConstants.kVI)
                .d(Constants.shooterConstants.kVD)
                .outputRange(-1, 1);
        m_motorMotorConfig.idleMode(IdleMode.kBrake);
        m_motor.configure(m_motorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}