package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Gripper extends SubsystemBase {

    DoubleSolenoid gripperDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public Gripper(){
    }

    public void setOff(){
        gripperDoublePCM.set(DoubleSolenoid.Value.kOff);
    }

    public void setForward(){
        gripperDoublePCM.set(DoubleSolenoid.Value.kForward);
    }

    public void setReverse(){
        gripperDoublePCM.set(DoubleSolenoid.Value.kReverse);
    }
}
