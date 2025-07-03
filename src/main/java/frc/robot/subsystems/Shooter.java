package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.GlobalConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
 

public class Shooter extends SubsystemBase {

    private final SparkMax m_shootermotor1 = new SparkMax(6, MotorType.kBrushless);
    private final SparkMax m_shootermotor2 = new SparkMax(5, MotorType.kBrushless);
    private final SparkMax m_feedermotor = new SparkMax(9, MotorType.kBrushless);


    private double m_desiredSpin = 0.0;

    private double m_desriedVel = 0.0;

    public Shooter() {

        configurePID();
        m_shootermotor1.getConfigurator().apply(slot0Configs);
        m_shootermotor2.getConfigurator().apply(slot0Configs);
        m_shootermotor1.getConfigurator().apply(CurrentLimit.kShooter);
        m_shootermotor2.getConfigurator().apply(CurrentLimit.kShooter);

        m_shootermotor1.setNeutralMode(NeutralModeValue.Brake);
        m_shootermotor2.setNeutralMode(NeutralModeValue.Brake);

        m_feedermotor
            .smartCurrentLimit();
    }

}

    public void configurePID() {
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.10; // An error of 1 rps results in 0.10 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    @Override
    public void periodic() {
        m_motor1.setControl(m_request.withVelocity(m_desriedVel + m_desiredSpin / 2.0).withSlot(0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * (m_desriedVel - m_desiredSpin / 2.0)).withSlot(0));

    }

    public void run(double velocity) {
        m_desriedVel = velocity;
        m_motor1.setControl(m_request.withVelocity(velocity).withSlot(0));
        m_motor2.setControl(m_request.withVelocity(-1.0 * velocity).withSlot(0));

    }

    public Command changeSpeed(double adjust) {
        return runOnce(() -> {
            m_desriedVel += adjust;
            if (m_desriedVel >= 80.0) {
                m_desriedVel = 80.0;
            } else if (m_desriedVel <= 10.0) {
                m_desriedVel = 10.0;
            }
        });
    }

    public void run(double velocity, double spinDiff) {
        spinDiff = 0.01 * spinDiff * velocity;
        if (velocity >= 100.0) {
            velocity = 100.0;
        } else if (velocity <= -20.0) {
            velocity = -20.0;
        }
        m_desriedVel = velocity;
        m_desiredSpin = spinDiff;
    }

    public Command runCommand(double velocity, double spinDiff) {
        return runEnd(() -> run(velocity, spinDiff), () -> stopShooter());
    }

    public void stopShooter() {
        m_shootermotor1.stopMotor();
        m_shootermotor2.stopMotor();
        m_desriedVel = 0.0;
        m_desiredSpin = 0.0;
    }

    public boolean atSetpoint() {
        return Math.abs(m_shootermotor1.getVelocity().getValueAsDouble() - m_desriedVel) <= 5.0;
    }

    public double getSetVelocity() {
        return m_desriedVel;
    }
    public void runFeeder(double speed) {
        m_feedermotor.set(speed);
    }

    public Command runCommand(double speed) {
        return runOnce(() -> run(speed));
    }

    public Command feed(){
        return runEnd(()->run(0.8), ()->stopFeeder());
    }

    public void stopFeeder() {
        m_feedermotor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(() -> stopFeeder());
    }
}


