package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.shooterHood;

public class ResetCommand extends Command{
    private final shooterHood m_hood;
    private final Shooter m_shooter;
    private final Intake m_intake;

    public ResetCommand(Shooter shooter, shooterHood hood, Intake intake) {
        m_shooter = shooter;
        m_hood = hood;
        m_intake = intake;
        addRequirements(shooter, hood, intake);
    }

    @Override
    public void initialize() {
        m_shooter.setVelocity(Constants.shooterConstants.kDefault);
        m_hood.setAngle(Constants.ShoulderConstants.kDefault);
        //m_intake.setVelocity(Constants.intakeConstants.kDefault);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
