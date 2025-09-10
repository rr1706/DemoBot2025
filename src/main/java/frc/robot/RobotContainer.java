package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  
    m_driverController.rightBumper().onTrue(m_Intake.intakeOutCommand2());
    // Intake In Command is set for when Right Bumber is not pressed 

    m_driverController.rightTrigger().whileTrue(m_Shooter.ShootCommand()).onFalse(m_Shooter.ShootCommandStop());
    // Shoot command is set for when left trigger is pressed

    m_driverController.povUp().onTrue(m_Shooter.changePitch(5));

    m_driverController.povDown().onFalse(m_Shooter.changePitch(-5));

    m_driverController.leftTrigger().onTrue(m_Shooter.ShootAngleCommand());

  }

  public double getIntakeAngle() {
    return m_Intake.getAngle();
      // Gets current angle the Intake is at.
  }

  public double getIntakeVelocity() {
    return m_Intake.getVelocity();
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
