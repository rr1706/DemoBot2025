package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shoulder;

public class IntakeCommand extends Command {
    private final Manipulator m_manipulator;
    private final Shoulder m_shoulder;
    private final double tol = 1.0;

    private boolean isReady() {
        if (Math.abs(m_shoulder.getPosition() - Constants.ShoulderConstants.kShoot) <= tol) {
            return true;
        } else {
            return false;
        }
    }

    private boolean Ball() {
        return m_manipulator.hasBall();
    }

    public IntakeCommand(Manipulator manipulator, Shoulder shoulder) {
        addRequirements(manipulator, shoulder);
        m_manipulator = manipulator;
        m_shoulder = shoulder;
    }

    @Override
    public void initialize() {
        m_shoulder.setAngle(Constants.ShoulderConstants.kIntake);
        m_manipulator.setVelocity(Constants.shooterConstants.kIntake);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
