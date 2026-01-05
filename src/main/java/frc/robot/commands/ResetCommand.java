package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shoulder;

public class ResetCommand extends Command{
    private final Shoulder m_shoulder;
    private final Manipulator m_manipulator;

    public ResetCommand(Manipulator manipulator, Shoulder shoulder) {
        m_manipulator = manipulator;
        m_shoulder = shoulder;
        addRequirements(manipulator, shoulder);
    }

    @Override
    public void initialize() {
        m_manipulator.setVelocity(Constants.shooterConstants.kDefault);
        m_shoulder.setAngle(Constants.ShoulderConstants.kDefault, 0);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
