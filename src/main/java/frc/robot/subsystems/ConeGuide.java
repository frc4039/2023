package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ConeGuide extends SubsystemBase {

    DoubleSolenoid coneGuidePneumatic = new DoubleSolenoid(Constants.solenoidCanID,PneumaticsModuleType.REVPH, 
                                                            Constants.ConeGuideConstants.kConeGuideForwardChannel, 
                                                            Constants.ConeGuideConstants.kConeGuideReverseChannel);

    public ConeGuide(){
    }

    public void setOff(){
        coneGuidePneumatic.set(DoubleSolenoid.Value.kOff);
    }

    public void setForward(){
        coneGuidePneumatic.set(DoubleSolenoid.Value.kForward);
    }

    public void setReverse(){
        coneGuidePneumatic.set(DoubleSolenoid.Value.kReverse);
    }
}