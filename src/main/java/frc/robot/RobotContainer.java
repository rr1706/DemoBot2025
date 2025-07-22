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

public class RobotContainer {
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
  }

  private void configureBindings() {
  
    m_driverController.rightBumper().and(m_driverController.a()).whileTrue(m_Intake.intakeOutCommand2());
    // Intake In Command is set for when Right Bumber is not pressed 
  }

  public double getIntakeAngle() {
    return m_Intake.getAngle();
      // Gets current angle the Intake is at.
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
     // Voids the Auto Command.
  }
}
