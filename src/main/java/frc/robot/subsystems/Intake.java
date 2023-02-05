package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {

    DoubleSolenoid intakePneumatic = new DoubleSolenoid(7,PneumaticsModuleType.REVPH, 4, 5);

    public Intake(){
    }

    public void setOff(){
        intakePneumatic.set(DoubleSolenoid.Value.kOff);
    }

    public void setDeploy(){
        intakePneumatic.set(DoubleSolenoid.Value.kForward);
    }

    public void setRetract(){
        intakePneumatic.set(DoubleSolenoid.Value.kReverse);
    }
}