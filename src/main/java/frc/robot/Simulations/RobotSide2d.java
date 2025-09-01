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

    private static double IntakeLenght = Units.inchesToMeters(24);
    private static double RollerLenght = Units.inchesToMeters(5);

    private final Mechanism2d m_mech2d = new Mechanism2d(DisplayLenght, DisplayHeight);
    // Create Robot Frame root point 8in form the left side and 2in off the ground.
    private final MechanismRoot2d mFrame2dRoot = m_mech2d.getRoot("Frame Root", FrameOffsetLength, FrameOffsetHeight);
    // Create Robot Frame with lenght of the robot.
    private final MechanismLigament2d m_Frame = mFrame2dRoot.append(new MechanismLigament2d("Frame", RobotFrameLength, 0, 48, new Color8Bit(Color.kBlue)));

    // Create Elevator root point.
    private final MechanismRoot2d m_Intake2dRootUpper = m_mech2d.getRoot("Intake Root Upper", (RobotFrameLength - Units.inchesToMeters(4)), (FrameOffsetHeight+RobotFrameHeight +Units.inchesToMeters(2)));
    private final MechanismLigament2d m_Intake2dUpper = m_Intake2dRootUpper.append(new MechanismLigament2d("Intake Upper", IntakeLenght, 90, 6.0, new Color8Bit(Color.kAliceBlue)));
    private final MechanismRoot2d m_IntakeRootRoller = m_mech2d.getRoot("Intake Root Roller", (RobotFrameLength - Units.inchesToMeters(4)), (FrameOffsetHeight+RobotFrameHeight +Units.inchesToMeters(24)));
    private final MechanismLigament2d m_IntakeRoller = m_IntakeRootRoller.append(new MechanismLigament2d("Intake Roller", RollerLenght, 90, 8.5, new Color8Bit(Color.kRed)));

    private final MechanismRoot2d m_Intake2dRootLower = m_mech2d.getRoot("Intake Root Lower", (RobotFrameLength - Units.inchesToMeters(2)), (FrameOffsetHeight+RobotFrameHeight +Units.inchesToMeters(0)));
    private final MechanismLigament2d m_Intake2dLower = m_Intake2dRootLower.append(new MechanismLigament2d("Intake Lower", IntakeLenght, 90, 6.0, new Color8Bit(Color.kAliceBlue)));
    
    public RobotSide2d() {
        // Publish Mechanism2d to SmartDashboard
        // To view the elevator visulization, select Network Tables -> SmartDashboard -> Elevator Sim
        SmartDashboard.putData("Robot Sim", m_mech2d);
    }

    public void updateDisplayPose(double IntakeAngle){

        m_Intake2dUpper.setAngle(90-Units.rotationsToDegrees(IntakeAngle));
        m_Intake2dLower.setAngle(90-Units.rotationsToDegrees(IntakeAngle));
    }

    public void updateDisplayPose2(double IntakeVelocity) {
        m_IntakeRoller.setAngle(90-Units.rotationsToDegrees(IntakeVelocity));
    }

}
