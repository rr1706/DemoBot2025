package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shoulder;

public class ShootCommand extends Command {
    private final Manipulator m_manipulator;
    private final Shoulder m_shoulder;
    private final double tol = 1.0;
    
    public ShootCommand(Manipulator manipulator, Shoulder shoulder) {
        addRequirements(shoulder, manipulator);
        m_shoulder = shoulder;
        m_manipulator = manipulator;
    }

    @Override
    public void initialize() {
        m_shoulder.setAngle(Constants.ShoulderConstants.kShoot);
        m_manipulator.setVelocity(5000);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {}
}
