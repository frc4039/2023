package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Gripper extends SubsystemBase {

    private DoubleSolenoid gripperPneumatic = new DoubleSolenoid(Constants.solenoidCanID,PneumaticsModuleType.REVPH, 0, 1);
    private TalonSRX m_tosserMotor;

    public Gripper(){
        m_tosserMotor = new TalonSRX(Constants.GripperConstants.tosserMotorID);
        m_tosserMotor.configFactoryDefault();        
    }

    public void setOff(){
        gripperPneumatic.set(DoubleSolenoid.Value.kOff);
        m_tosserMotor.set(TalonSRXControlMode.PercentOutput,0);
    }

    public void setForward(){
        gripperPneumatic.set(DoubleSolenoid.Value.kForward);
        m_tosserMotor.set(TalonSRXControlMode.PercentOutput, Constants.GripperConstants.tosserMotorForwardPercent);
    }

    public void setReverse(){
        gripperPneumatic.set(DoubleSolenoid.Value.kReverse);
        m_tosserMotor.set(TalonSRXControlMode.PercentOutput, Constants.GripperConstants.tosserMotorReversePercent);
    }
}
