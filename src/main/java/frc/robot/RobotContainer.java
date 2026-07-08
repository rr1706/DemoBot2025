package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveByController;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final Shooter m_shooter = new Shooter();
  private final ShooterHood m_shooterHood = new ShooterHood();
  private final Intake m_intake = new Intake();
  private final Drivetrain m_driveTrain = new Drivetrain();

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Center");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    m_driveTrain.setDefaultCommand(new DriveByController(m_driveTrain, m_driverController));
  }

  private void configureBindings() {
    m_driverController.rightTrigger().whileTrue(m_shooter.shootCmd().alongWith(m_shooterHood.shootCmd()));

    m_driverController.leftTrigger().whileTrue(m_intake.intakeCmd());

    m_driverController.a().onTrue(new InstantCommand(()-> m_driveTrain.resetOdometry(new Pose2d())));

    m_driverController.povUp().onTrue(new InstantCommand(()-> m_shooterHood.setAngle(m_shooterHood.changePitch(5))));
    m_driverController.povDown().onTrue(new InstantCommand(()-> m_shooterHood.setAngle(m_shooterHood.changePitch(-5))));

    m_driverController.povLeft().onTrue(new InstantCommand(()-> m_shooter.setVelocity(m_shooter.changeVelocity(5))));
    m_driverController.povRight().onTrue(new InstantCommand (()-> m_shooter.setVelocity(m_shooter.changeVelocity(-5))));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
