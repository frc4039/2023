package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Gripper extends SubsystemBase {

    // forwardChannel = 0 => closes gripper
    // reverseChannel = 1 => opens gripper
    private DoubleSolenoid gripperPneumatic = new DoubleSolenoid(Constants.kSolenoidCanID, PneumaticsModuleType.REVPH,
            Constants.GripperConstants.kGripperForwardChannel,
            Constants.GripperConstants.kGripperReverseChannel);

    public Gripper() {
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
}
