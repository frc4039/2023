package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Gripper extends SubsystemBase {

    DoubleSolenoid gripperDoublePCM = new DoubleSolenoid(7,PneumaticsModuleType.REVPH, 0, 1);
    int solenoid_0 = 0;
    int solenoid_1 = 0;

    public Gripper(){
    }

    public void setOff(){
        gripperDoublePCM.set(DoubleSolenoid.Value.kOff);
    }

    public void setForward(){
        gripperDoublePCM.set(DoubleSolenoid.Value.kForward);
        solenoid_0 = 1;
        solenoid_1 = 0;
    }

    public void setReverse(){
        gripperDoublePCM.set(DoubleSolenoid.Value.kReverse);
        solenoid_0 = 0;
        solenoid_1 = 1;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("gripper encoder", solenoid_0);
        SmartDashboard.putNumber("target gripper position",  solenoid_1);
    }
}
