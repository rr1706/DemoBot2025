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
import frc.robot.subsystems.shooterHood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final Shooter m_Shooter = new Shooter();
  private final shooterHood m_pitcher = new shooterHood();
  private final Intake m_intake = new Intake();
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
    m_driverController.rightTrigger().onTrue(new ShootCommand(m_Shooter, m_pitcher))
                                      .onFalse(new ResetCommand(m_Shooter, m_pitcher, m_intake));

    m_driverController.leftTrigger().onTrue(new IntakeCommand(m_intake, m_pitcher))
                                     .onFalse(new ResetCommand(m_Shooter, m_pitcher, m_intake));

    m_driverController.a().onTrue(new InstantCommand(()-> m_driveTrain.resetOdometry(new Pose2d())));

    m_driverController.povUp().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(5))));
    m_driverController.povDown().onTrue(new InstantCommand(()-> m_pitcher.setAngle(m_pitcher.changePitch(-5))));

    m_driverController.povLeft().onTrue(new InstantCommand(()-> m_Shooter.setVelocity(m_Shooter.changeVelocity(5))));
    m_driverController.povRight().onTrue(new InstantCommand (()-> m_Shooter.setVelocity(m_Shooter.changeVelocity(-5))));
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