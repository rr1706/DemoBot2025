package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.shooterHood;

public class IntakeCommand extends Command {
    private final Intake m_intake;
    private final shooterHood m_hood;
    private final Shooter m_Shooter;

    public IntakeCommand(Intake intake, shooterHood hood, Shooter shooter) {
        addRequirements(intake, hood);
        m_intake = intake;
        m_hood = hood;
        m_Shooter = shooter;
    }

    @Override
    public void initialize() {
        m_hood.setAngle(Constants.ShoulderConstants.kZero);
        m_intake.intake();
        m_Shooter.setVelocity(Constants.shooterConstants.kRun);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
