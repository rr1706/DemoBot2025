package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.subsystems.ExampleSubsystem;
import javax.lang.model.util.ElementScanner14;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
  
    m_driverController.rightBumper().and(m_driverController.a()).whileTrue(m_Intake.intakeOutCommand()).onFalse(m_Intake.intakeInCommand());
    // Intake In Command is set for when Right Bumber is not pressed 

    m_driverController.leftTrigger().whileTrue(m_Shooter.ShootCommand()).onFalse(m_Shooter.ShootCommandStop());
    // Shoot command is set for when left trigger is pressed

    m_driverController.povUp().whileTrue(m_Shooter.ShooterAngleAjustUpCommand());

    m_driverController.povDown().whileTrue(m_Shooter.ShooterAngleAjustDownCommand());
  }

  public double getIntakeAngle() {
    return m_Intake.getAngle();
      // Gets current angle the Intake is at.
  }

  public double getShooterVelocity() {
    return m_Shooter.getVelocity();
  }

  public double getShooterAngle() {
    return m_Shooter.getPosition();
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
     // Voids the Auto Command.
  }
}
