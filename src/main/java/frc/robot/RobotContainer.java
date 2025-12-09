package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveByController;
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

  private void configureDefaultCommands() {}

  private void configureBindings() {
    m_driverController.leftTrigger().onTrue(new InstantCommand(()-> m_shooter.setVelocity(Constants.shooterConstants.kVelocity))).onFalse(new InstantCommand(()-> m_shooter.setVelocity(0)));
    m_driverController.rightTrigger().onTrue(new InstantCommand(()-> m_shooter.setVelocity(Constants.shooterConstants.kVelocityIntake))).onFalse(new InstantCommand(()-> m_shooter.setVelocity(0)));

    m_driverController.leftBumper().onTrue(new InstantCommand(()-> m_pitcher.setAngle(40, 8.0)))
                                  .onFalse(new InstantCommand(()-> m_pitcher.setAngle(20, 8.0)));

    m_driverController.povUp().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(5), 1)));
    m_driverController.povDown().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(-5), 1)));

    m_driverController.povLeft().onTrue(new InstantCommand(()-> m_shooter.changeVelocity(5)));
    m_driverController.povRight().onTrue(new InstantCommand (()-> m_shooter.changeVelocity(-5)));
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
