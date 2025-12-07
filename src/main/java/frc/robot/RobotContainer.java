package frc.robot;

import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AngleSubsystem m_pitcher = new AngleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftTrigger().whileTrue(m_shooter.IntakeCommand()).onFalse(m_shooter.IntakeCommandStop());

    m_driverController.rightTrigger().whileTrue(m_shooter.ShootCommand()).onFalse(m_shooter.ShootCommandStop());
    // Shoot command is set for when left trigger is pressed

    m_driverController.povUp().onTrue(m_pitcher.changePitch(5));

    m_driverController.povDown().onTrue(m_pitcher.changePitch(-5));

    m_driverController.a().onTrue(m_pitcher.ShootAngleCommand());

    m_driverController.leftBumper.onTrue(m_shooter.changeVelocity(5));
    m_driverController.rightBumper.onTrue(m_shooter.changeVelocity(-5));
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
