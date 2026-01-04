package frc.robot.Simulations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotSide2d {
    private static double RobotFrameLength = Units.inchesToMeters(24);
    private static double RobotFrameHeight = Units.inchesToMeters(2);

    private static double DisplayLenght = Units.inchesToMeters(50);
    private static double DisplayHeight = Units.inchesToMeters(2);

    private static double FrameOffsetLength = (DisplayLenght-RobotFrameLength)/2;
    private static double FrameOffsetHeight = Units.inchesToMeters(2);

    private static double PitcherLenght = Units.inchesToMeters(10);

    private double m_setAngle = 90;
    private double m_trueAngle = 90;

    public void pitcherAngle(double setAngle) {
        m_setAngle = setAngle;
    }

    public void pitcherRealAngle(double angle) {
        m_trueAngle = angle;
    }

    private final Mechanism2d m_mech2d = new Mechanism2d(DisplayLenght, DisplayHeight);
    // Create Robot Frame root point 8in form the left side and 2in off the ground.
    private final MechanismRoot2d mFrame2dRoot = m_mech2d.getRoot("Frame Root", FrameOffsetLength, FrameOffsetHeight);
    // Create Robot Frame with lenght of the robot.
    private final MechanismLigament2d m_Frame = mFrame2dRoot.append(new MechanismLigament2d("Frame", RobotFrameLength, 0, 48, new Color8Bit(Color.kBlue)));

    // Create Elevator root point.
    private final MechanismRoot2d m_pitcher2dRoot = m_mech2d.getRoot("pitcher Root Upper", (RobotFrameLength - Units.inchesToMeters(4)), (FrameOffsetHeight+RobotFrameHeight +Units.inchesToMeters(2)));
    private final MechanismLigament2d m_pitcher2d = m_pitcher2dRoot.append(new MechanismLigament2d("pitcher Upper", PitcherLenght, m_setAngle, 6.0, new Color8Bit(Color.kAliceBlue)));

    private final MechanismRoot2d m_pitcherSet2dRoot = m_mech2d.getRoot("pitcher Root Lower", (RobotFrameLength - Units.inchesToMeters(4)), (FrameOffsetHeight+RobotFrameHeight +Units.inchesToMeters(2)));
    private final MechanismLigament2d m_pitcherSet2d = m_pitcherSet2dRoot.append(new MechanismLigament2d("pitcher Lower", PitcherLenght, m_trueAngle, 6.0, new Color8Bit(Color.kRed)));

    public RobotSide2d() {
        SmartDashboard.putData("Robot Sim", m_mech2d);
    }
}
