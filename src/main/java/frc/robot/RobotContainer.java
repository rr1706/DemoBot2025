package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Simulations.RobotSide2d;
import frc.robot.commands.DriveByController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final Manipulator m_manipulator = new Manipulator();
  private final Shoulder m_pitcher = new Shoulder();
  private final Drivetrain m_driveTrain = new Drivetrain();
  private final RobotSide2d m_simulation = new RobotSide2d();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_fullController =
      new CommandXboxController(OperatorConstants.kFullPort);

  private boolean controllerInUse = false;
  
  public RobotContainer() {
    configureBindings();
    if (controllerInUse) {
      m_driveTrain.setDefaultCommand(new DriveByController(m_driveTrain, m_driverController));
    } else {
      m_driveTrain.setDefaultCommand(new DriveByController(m_driveTrain, m_fullController));
    }
  }

  private void configureBindings() {
    m_driverController.leftTrigger().onTrue(new IntakeCommand(m_manipulator, m_pitcher));

    m_driverController.rightTrigger().onTrue(new ShootCommand(m_manipulator, m_pitcher));

        //For finner control set the controller port to 1 in DriveStation.
    m_fullController.a().onTrue(new InstantCommand(()-> m_driveTrain.resetOdometry(new Pose2d())));
    
    m_fullController.leftTrigger().onTrue(new InstantCommand(()-> m_manipulator.Intake()))
                                  .onFalse(new InstantCommand(()-> m_manipulator.Stop()));
    m_fullController.rightTrigger().onTrue(new InstantCommand(()-> m_manipulator.Shoot()))
                                   .onFalse(new InstantCommand(()-> m_manipulator.Stop()));

    m_fullController.leftBumper().onTrue(new InstantCommand(()-> m_pitcher.Intake()))
                                  .onFalse(new InstantCommand(()-> m_pitcher.Home()));
    m_fullController.rightBumper().onTrue(new InstantCommand(()-> m_pitcher.Shoot()))
                                  .onFalse(new InstantCommand(()-> m_pitcher.Home()));

    m_fullController.povUp().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(5), 0)));
    m_fullController.povDown().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(-5), 0)));

    m_fullController.povLeft().onTrue(new InstantCommand(()-> m_manipulator.setVelocity(m_manipulator.changeVelocity(5))));
    m_fullController.povRight().onTrue(new InstantCommand (()-> m_manipulator.setVelocity(m_manipulator.changeVelocity(-5))));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0.0);
     // Voids the Auto Command.
  }

  public void updateSims() {
    m_simulation.pitcherAngle(m_pitcher.getPosition());
  }
}
