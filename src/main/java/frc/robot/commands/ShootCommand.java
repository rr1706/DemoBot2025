package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.shooterHood;

public class ShootCommand extends Command {
    private final Shooter m_shooter;
    private final shooterHood m_hood;
    
    public ShootCommand(Shooter shooter, shooterHood hood) {
        addRequirements(shooter, hood);
        m_shooter = shooter;
        m_hood = hood;
    }

    @Override
    public void initialize() {
        m_hood.setAngle(Constants.ShoulderConstants.kShoot);
        m_shooter.setVelocity(Constants.shooterConstants.kShoot);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}
}
