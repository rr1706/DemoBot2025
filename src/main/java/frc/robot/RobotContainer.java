package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveByController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.simulations.RobotSide2d;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SendableChooser<Command> autoChooser;


  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_fullController =
      new CommandXboxController(OperatorConstants.kFullPort);
  
  public RobotContainer() {
     autoChooser = AutoBuilder.buildAutoChooser("Center");
      SmartDashboard.putData("Auto Mode", autoChooser);
      configureBindings();
      m_driveTrain.setDefaultCommand(new DriveByController(m_driveTrain, m_driverController));
}

  private void configureBindings() {
    m_driverController.rightTrigger().onTrue(new ShootCommand(m_manipulator, m_pitcher))
                                      .onFalse(new ResetCommand(m_manipulator, m_pitcher));

    m_driverController.leftTrigger().onTrue(new IntakeCommand(m_manipulator, m_pitcher))
                                     .onFalse(new ResetCommand(m_manipulator, m_pitcher));

    m_driverController.a().onTrue(new InstantCommand(()-> m_driveTrain.resetOdometry(new Pose2d())));

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

    m_driverController.povUp().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(5))));
    m_driverController.povDown().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(-5))));

    m_driverController.povLeft().onTrue(new InstantCommand(()-> m_manipulator.setVelocity(m_manipulator.changeVelocity(5))));
    m_driverController.povRight().onTrue(new InstantCommand (()-> m_manipulator.setVelocity(m_manipulator.changeVelocity(-5))));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // Voids the Auto Command.
  }
 
  public double getPitcherPose() {
    return m_pitcher.getPosition();
  }

  public double getPitcherSetPose() {
    return m_pitcher.getSetAngle();
  }
}