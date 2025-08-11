package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ModuleConstants.Drive;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


/*private final CANSparkMax m_motor1;
private final CANSparkMax m_motor2;
private final RelativeEncoder m_encoder1;
private final RelativeEncoder m_encoder2;
private final PIDController m_PID = new PIDController(ShooterConstants.kPID[0], ShooterConstants.kPID[1],
        ShooterConstants.kPID[2]);
private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(ShooterConstants.kStatic, ShooterConstants.kFF);

private double m_RPM = ShooterConstants.kMaxRPM;*/

public class Shooter extends SubsystemBase {
    private final SparkMax m_shootermotor1;
    private final SparkMax m_shootermotor2;
    private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(ShooterConstants.kStatic, ShooterConstants.kFF);    private final PIDController m_PID = new PIDController(Constants.ShooterConstants.kPID[0], Constants.ShooterConstants.kPID[1], 
        Constants.ShooterConstants.kPID[2]);
    private final SparkMaxConfig m_motorConfigSM1 = new SparkMaxConfig();
    public final RelativeEncoder m_shooterM1Enc;
    private final SparkMaxConfig m_motorConfigSM2 = new SparkMaxConfig();
    public final RelativeEncoder m_shooterM2Enc;
    private double m_RPM = Constants.ShooterConstants.kMaxRPM;


    public Shooter(int moduleID, double offset) {
        m_shootermotor1 = new SparkMax(moduleID+1, MotorType.kBrushless);
        m_shootermotor2 = new SparkMax(moduleID, MotorType.kBrushless);
        m_shooterM1Enc = m_shootermotor1.getEncoder();
        m_shooterM2Enc = m_shootermotor2.getEncoder();

        m_motorConfigSM1
            .smartCurrentLimit(CurrentLimit.kShooter)
            .voltageCompensation(GlobalConstants.kVoltageCompensation)
            .idleMode(IdleMode.kCoast);

        m_motorConfigSM2
            .smartCurrentLimit(CurrentLimit.kShooter)
            .voltageCompensation(GlobalConstants.kVoltageCompensation)
            .idleMode(IdleMode.kCoast);
            
        m_shootermotor2.isFollower(); //look up way to make motor follower

        m_shooterM1Enc.velocityConversionFactor(1.0);

        m_shootermotor1.configure(m_motorConfigSM1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters) ;//find new method for burn flash
        m_shootermotor2.configure(m_motorConfigSM2,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters) ;//find new method for burn flash


        m_PID.setTolerance(Constants.ShooterConstants.kRPMTolerance);

        m_PID.setIntegratorRange(-Constants.ShooterConstants.kIntRange, Constants.ShooterConstants.kIntRange);

}
        public void run(double rpm) {
        if (rpm >= ShooterConstants.kMaxRPM) {
            rpm = ShooterConstants.kMaxRPM;
        }
        m_RPM = rpm;
        double outputPID = m_PID.calculate(m_shooterM1Enc.getVelocity(), m_RPM);
        double outputFF = m_FF.calculate(m_RPM);
        double output = outputPID + outputFF;

        if (output <= ShooterConstants.kMaxNegPower) {
            output = ShooterConstants.kMaxNegPower;
        }

        m_shootermotor1.set(outputPID + outputFF);
    }

    public void stop() {
        m_shootermotor1.stopMotor();
        m_shootermotor2.stopMotor();
}
}


