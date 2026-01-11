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

    private DoubleTopic m_rollerAngleTopic = NetworkTableInstance.getDefault().getTable("Roller")
            .getDoubleTopic("/Roller/Velocity");
    private DoublePublisher m_rollerAnglePublish = m_rollerAngleTopic.publish();

    private final SparkMax m_roller = new SparkMax(m_rollerPort, MotorType.kBrushless);
    // Roller Motor

    private final SparkMaxConfig m_rollerMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private final SparkClosedLoopController m_rollerPID;

    private final static int m_rollerPort = 9;
    // Roller port, could be changed.

    private final static double m_rollerGearing = 1;
    // Gearing for the Rotation and Roller motors TBC.

    private final RelativeEncoder m_rollerEncoder = m_roller.getEncoder();

    public ShooterSubsystem() {
        m_rollerPID = m_roller.getClosedLoopController();


        m_rollerMotorConfig.encoder
                .positionConversionFactor(m_rollerGearing)
                .velocityConversionFactor(m_rollerGearing / 60);

        m_rollerMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1);
        m_roller.configure(m_rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        m_rollerAnglePublish.set(Units.rotationsToDegrees(getVelocity()));
                // Converts to correct unit then published to SmartDashboard.
    }

    public void ShooterRoller(double target) {
        m_rollerPID.setReference(Units.degreesToRotations(target), ControlType.kVelocity);
        // Allows you to change target value later in code for the roller motor.
    }

    public double changeVelocity(double input) {
        double currentVelocity = getVelocity();
        double newVelocity = currentVelocity + input;
        return newVelocity;
    }

    public double getVelocity() {
        double Velocity = m_rollerEncoder.getPosition();
        SmartDashboard.getNumber("Roller Velocity", Velocity);
        return Velocity;
    }
}