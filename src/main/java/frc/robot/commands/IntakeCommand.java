package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooterHood;

public class IntakeCommand extends Command {
    private final Intake m_intake;
    private final shooterHood m_hood;
    private final double tol = 1.0;

    public IntakeCommand(Intake intake, shooterHood hood) {
        addRequirements(intake, hood);
        m_intake = intake;
        m_hood = hood;
    }

    @Override
    public void initialize() {
        m_hood.setAngle(Constants.ShoulderConstants.kIntake);
        //m_intake.setVelocity(Constants.shooterConstants.kIntake);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
