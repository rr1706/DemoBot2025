package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AngleSubsystem m_pitcher = new AngleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    m_pitcher.setDefaultCommand(m_pitcher.ShootAngleCommand());
  }

  private void configureBindings() {
    m_driverController.leftTrigger().whileTrue(new InstantCommand(()-> m_shooter.setVelocity(Constants.shooterConstants.kVelocity)));

    m_driverController.rightTrigger().whileTrue(new InstantCommand(()-> m_shooter.setVelocity(Constants.shooterConstants.kVelocityIntake)));

    m_driverController.povUp().onTrue(m_pitcher.changePitch(5));
    m_driverController.povDown().onTrue(m_pitcher.changePitch(-5));

    m_driverController.leftBumper().onTrue(new InstantCommand(()-> m_shooter.changeVelocity(5)));
    m_driverController.rightBumper().onTrue(new InstantCommand (()-> m_shooter.changeVelocity(-5)));
  }

  public double getShooterVelocity() {
    return m_shooter.getVelocity();
  }

  public double getAngle() {
    return m_pitcher.getPosition();
  }

  public double getSetAngle() {
    return m_pitcher.getSetAngle();
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
     // Voids the Auto Command.
  }
}
