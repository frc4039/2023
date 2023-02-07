package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase {

    DoubleSolenoid intakePneumatic = new DoubleSolenoid(7,PneumaticsModuleType.REVPH, 4, 5);
    VictorSPX spinningHexMotor = new VictorSPX(IntakeConstants.spinningIntakeMotorID); // CANID 42...
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

    public void setSpinningMotorOn(){
        spinningHexMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.intakeSpinningMotorForward);
    }

    public void setSpinningMotorOff(){
        spinningHexMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.intakeSpinningMotorOff);
    }
}