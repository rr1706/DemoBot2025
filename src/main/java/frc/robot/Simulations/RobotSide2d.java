package frc.robot.simulations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotSide2d {
    private double m_setAngle = 90;
    private double m_trueAngle = 90;

    public void pitcherAngle(double setAngle, double trueAngle) {
        m_setAngle = setAngle;
        m_trueAngle = trueAngle;
    }

    private final Mechanism2d pitcherMech = new Mechanism2d(0, 0);
    private final MechanismRoot2d pitcherRoot = pitcherMech.getRoot("Pitcher Root", 1, 0);
    private final MechanismLigament2d pitcherSet = pitcherRoot.append(new MechanismLigament2d("Pitcher Set", 1, m_setAngle, 6.0, new Color8Bit(Color.kAliceBlue)));
    private final MechanismLigament2d pitcherTrue = pitcherRoot.append(new MechanismLigament2d("Pitcher True", 1, m_setAngle, 6.0, new Color8Bit(Color.kRed)));


    public RobotSide2d() {
        SmartDashboard.putData("Pitcher Sim", pitcherMech);
    }
}
