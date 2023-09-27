package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.GripperConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Gripper extends SubsystemBase {

    // forwardChannel = 0 => closes gripper
    // reverseChannel = 1 => opens gripper
    private DoubleSolenoid gripperPneumatic = new DoubleSolenoid(Constants.kSolenoidCanID, PneumaticsModuleType.REVPH,
            Constants.GripperConstants.kGripperForwardChannel,
            Constants.GripperConstants.kGripperReverseChannel);
    private DigitalInput m_BeamBreaker;
    private boolean m_ReleaseMode;
    private BlinkinGamePiece m_BlinkinGamePiece;
    private boolean m_UseBeamBreaker;

    public Gripper(BlinkinGamePiece blinkinGamePiece) {
        m_BeamBreaker = new DigitalInput(GripperConstants.kBeamBreakerChannel);
        m_ReleaseMode = false;
        m_BlinkinGamePiece = blinkinGamePiece;
        m_UseBeamBreaker = true;
    }

    public void setOff() {
        gripperPneumatic.set(DoubleSolenoid.Value.kOff);
    }

    public void setClose() {
        gripperPneumatic.set(DoubleSolenoid.Value.kForward);
    }

    public void setOpen() {
        gripperPneumatic.set(DoubleSolenoid.Value.kReverse);
    }

    public Value getState() {
        return gripperPneumatic.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_UseBeamBreaker) {
            if (!m_ReleaseMode && !m_BeamBreaker.get()) {
                setClose();
                m_BlinkinGamePiece.SetColour(BlinkinConstants.kColourValueFlashing);
                m_ReleaseMode = true;
            }
            if (m_BeamBreaker.get()) {
                m_ReleaseMode = false;
            }
        }
    }
}
