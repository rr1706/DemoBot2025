package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveByController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AngleSubsystem m_pitcher = new AngleSubsystem();
  private final Drivetrain m_driveTrain = new Drivetrain();
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final DriveByController m_driveByController = new DriveByController(m_driveTrain, m_driverController);
  private final CommandJoystick m_driverControllerSim = new CommandJoystick(1);

  public RobotContainer() {
    configureBindings();
    m_driveTrain.setDefaultCommand(m_driveByController);
  }

  private void configureBindings() {
    m_driverController.leftTrigger().whileTrue(m_shooter.IntakeCommand())
                                    .onFalse(m_shooter.StopCommand());

    m_driverController.rightTrigger().whileTrue(m_shooter.ShootCommand())
                                    .onFalse(m_shooter.StopCommand());

    m_driverController.leftBumper().whileTrue(m_pitcher.IntakeCommand())
                                    .onFalse(m_pitcher.HomeCommand());

    m_driverController.povUp().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(5))));
    m_driverController.povDown().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(-5))));

    m_driverController.povLeft().onTrue(new InstantCommand(()-> m_shooter.setVelocity(m_shooter.changeVelocity(5))));
    m_driverController.povRight().onTrue(new InstantCommand (()-> m_shooter.setVelocity(m_shooter.changeVelocity(-5))));

    m_driverControllerSim.button(1).whileTrue(m_shooter.IntakeCommand()).onFalse(m_shooter.StopCommand());
    m_driverControllerSim.button(2).whileTrue(m_shooter.ShootCommand()).onFalse(m_shooter.StopCommand());
    m_driverControllerSim.button(3).whileTrue(m_pitcher.IntakeCommand()).onFalse(m_pitcher.HomeCommand());
    m_driverControllerSim.button(4).whileTrue(m_pitcher.ShootCommand()).onFalse(m_pitcher.HomeCommand());
    m_driverControllerSim.button(5).onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(5))));
    m_driverControllerSim.button(6).onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(-5))));
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
