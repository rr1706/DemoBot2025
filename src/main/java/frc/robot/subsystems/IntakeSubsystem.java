package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax m_rotate;
    private final SparkMax m_roller;
    private final SparkMaxConfig m_rotateMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_rollerMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_rotatePID;
    private final SparkClosedLoopController m_rollerPID;
    private final static int m_rotatePort = 1;
    private final static int m_rollerPort = 2;
    private final static double m_rotateGearing = 1;
    private final static double m_rollerGearing = 1;
    private final RelativeEncoder m_rotateEncoder;
    private final RelativeEncoder m_rollerEncoder;


    public IntakeSubsystem() {
      /*
       * Initialize the SPARK MAX and get its encoder and closed loop controller
       * objects for later use.
       */
      m_rotate = new SparkMax(m_rotatePort, MotorType.kBrushless);
      m_rotatePID = m_rotate.getClosedLoopController();
      m_rotateEncoder = m_rotate.getEncoder();


      m_rotateMotorConfig.encoder
      .positionConversionFactor(m_rotateGearing)
      .velocityConversionFactor(m_rotateGearing/60);

       m_rotateMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
        m_rotate.configure(m_rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      m_roller = new SparkMax(m_rollerPort, MotorType.kBrushless);
      m_rollerPID = m_roller.getClosedLoopController();
      m_rollerEncoder = m_roller.getEncoder();

        m_rollerMotorConfig.encoder
      .positionConversionFactor(m_rollerGearing)
      .velocityConversionFactor(m_rollerGearing/60);

       m_rotateMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
        m_roller.configure(m_rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    
    public void IntakeRotate(double target) {

        m_rotatePID.setReference(target, ControlType.kPosition);
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
    }
}

    public Command intakeOutCommand2() {
        return this.run(()->this.IntakeOut());
    }
}