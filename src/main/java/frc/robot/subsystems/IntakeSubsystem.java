package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private DoubleTopic m_rotateAngleTopic = NetworkTableInstance.getDefault().getTable("Rotate").getDoubleTopic("/Rotate/Angle");
    private DoublePublisher m_rotateAnglePublish = m_rotateAngleTopic.publish();

    private final SparkMax m_rotate = new SparkMax(m_rotatePort, MotorType.kBrushless);;
    // Rotation Motor
    private final SparkMax m_roller;
    // Roller Motor
    private final SparkMaxConfig m_rotateMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_rollerMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_rotatePID;
    private final SparkClosedLoopController m_rollerPID;
    private final static int m_rotatePort = 1;
    // Rotation Port, could be changed.
    private final static int m_rollerPort = 2;
    // Roller port, could be changed.
    private final static double m_rotateGearing = 25;
    private final static double m_rollerGearing = 1;
    private final RelativeEncoder m_rotateEncoder = m_rotate.getEncoder();
    private final RelativeEncoder m_rollerEncoder;

    private final SparkMaxSim m_rotateSim = new SparkMaxSim(m_rotate, DCMotor.getNEO(1));
    private final SparkRelativeEncoderSim m_rotateEncoderSim = new SparkRelativeEncoderSim(m_rotate);
    private final SingleJointedArmSim m_rotateArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), 
                                                                                m_rotateGearing,
                                                                                SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), Units.lbsToKilograms(4)),
                                                                                Units.inchesToMeters(12),
                                                                                0,
                                                                                Math.PI/2,
                                                                                true,
                                                                                Math.PI/2,
                                                                                0.0,0.0);
    
    public IntakeSubsystem() {

      m_rotatePID = m_rotate.getClosedLoopController();

      m_rotateMotorConfig.encoder
      .positionConversionFactor(m_rotateGearing)
      .velocityConversionFactor(m_rotateGearing/60);

       m_rotateMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.08)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
        m_rotateMotorConfig.idleMode(IdleMode.kBrake);
        m_rotate.configure(m_rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      m_roller = new SparkMax(m_rollerPort, MotorType.kBrushless);
      m_rollerPID = m_roller.getClosedLoopController();
      m_rollerEncoder = m_roller.getEncoder();

        m_rollerMotorConfig.encoder
      .positionConversionFactor(m_rollerGearing)
      .velocityConversionFactor(m_rollerGearing/60);

       m_rollerMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
        m_roller.configure(m_rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    
    @Override
    public void simulationPeriodic() {
        m_rotateArmSim.setInputVoltage(m_rotateSim.getAppliedOutput()*RobotController.getBatteryVoltage());
        m_rotateArmSim.update(0.02);
        m_rotateSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_rotateArmSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.2);
        m_rotateEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_rotateArmSim.getVelocityRadPerSec()), 0.2);
        // Intake Out.In Simulation

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_rotateArmSim.getCurrentDrawAmps()));
    }

    @Override
    public void periodic() {
        m_rotateAnglePublish.set(Units.rotationsToDegrees(getAngle()));
    }


    public void IntakeRotate(double target) {

        m_rotatePID.setReference(Units.degreesToRotations(-target), ControlType.kPosition);
    }

    public void IntakeRoller(double target) {

        m_rollerPID.setReference(target, ControlType.kVelocity);
    }

    public double getAngle() {
        return m_rotateEncoder.getPosition();
    }

    private static final double tol = 0.1;

    private void IntakeOut() {
    IntakeRotate(IntakeConstants.kOut);
     if (Math.abs(getAngle()-IntakeConstants.kOut) < tol) {
        IntakeRoller(IntakeConstants.kVelocity);
        // Sets volocity & Angle for Roller & Rotation Motors
    }

    }

    private void IntakeIn() {
     IntakeRotate(IntakeConstants.kIn);
     IntakeRoller(IntakeConstants.kVelocityIn);
     // Sets volocity & Angle for Roller & Rotation Motors

    }

    public Command intakeOutCommand2() {
        return this.run(()->this.IntakeOut());
        // Creates command for moving intake out.
    }

    public Command intakeInCommand() {
        return this.run(()->this.IntakeIn());
        // Creates command for moving intake in.
    }
}

