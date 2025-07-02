package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command {

    private final IntakeSubsystem m_Intake;

  public IntakeOutCommand(IntakeSubsystem subsystem) {
    m_Intake = subsystem;
    addRequirements(subsystem);

  }
  @Override
  public void initialize() {

    m_Intake.IntakeRotate(IntakeConstants.kOut);
  }
  @Override
  public void execute() {
    
    }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
