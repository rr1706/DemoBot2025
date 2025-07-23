package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private SparkMax m_Shooter = new SparkMax(m_shooterPort, MotorType.kBrushless);

    private final SparkMaxConfig m_shooterMotorConfig = new SparkMaxConfig();
    // Creates the motors configurations.

    private final SparkClosedLoopController m_shooterPID;

    private final static int m_shooterPort = 1;
    // Shooter Port, could be changed.

    private static double m_shooterGearing = 1;

    private final RelativeEncoder m_shooterEncoder = m_Shooter.getEncoder();

    public ShooterSubsystem() {

        m_shooterPID = m_Shooter.getClosedLoopController();

        m_shooterMotorConfig.encoder
          .positionConversionFactor(m_shooterGearing)
          .velocityConversionFactor(m_shooterGearing / 60);

        m_shooterMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0)
                .i(0)
                .d(0)
                .outputRange(-1, 1);
        m_shooterMotorConfig.idleMode(IdleMode.kBrake);
        m_Shooter.configure(m_shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

        public Command ShootCommand() {
            return this.runEnd(() -> {
                
            {m_shooterPID.setReference(Units.degreesToRotations(ShooterConstants.kVelocity), ControlType.kVelocity);}
                    
             }, () -> {m_shooterPID.setReference(Units.degreesToRotations(ShooterConstants.kVelocityStop), ControlType.kVelocity);});
                   
        }
    
}

