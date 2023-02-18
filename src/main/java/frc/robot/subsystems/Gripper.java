package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Gripper extends SubsystemBase {

    //forwardChannel = 0 => closes gripper
    //reverseChannel = 1 => opens gripper
    private DoubleSolenoid gripperPneumatic = new DoubleSolenoid(Constants.solenoidCanID,PneumaticsModuleType.REVPH, 
                                                                    Constants.GripperConstants.kGripperForwardChannel, 
                                                                    Constants.GripperConstants.kGripperReverseChannel);
    private TalonSRX m_tosserMotor;

    public Gripper(){
        m_tosserMotor = new TalonSRX(Constants.GripperConstants.tosserMotorID);
        m_tosserMotor.configFactoryDefault();        
    }

    public void setOff(){
        gripperPneumatic.set(DoubleSolenoid.Value.kOff);
        m_tosserMotor.set(TalonSRXControlMode.PercentOutput,0);
    }

    public void setClose(){
        gripperPneumatic.set(DoubleSolenoid.Value.kForward);
        m_tosserMotor.set(TalonSRXControlMode.PercentOutput, Constants.GripperConstants.tosserMotorForwardPercent);
    }

    public void setOpen(){
        gripperPneumatic.set(DoubleSolenoid.Value.kReverse);
        m_tosserMotor.set(TalonSRXControlMode.PercentOutput, Constants.GripperConstants.tosserMotorReversePercent);
    }
}
