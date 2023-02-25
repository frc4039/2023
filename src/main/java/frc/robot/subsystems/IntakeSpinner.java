package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.IntakeConstants;

public class IntakeSpinner extends SubsystemBase {

    private final VictorSPX m_spinningIntakeMotor;

    public IntakeSpinner() {
        m_spinningIntakeMotor = new VictorSPX(IntakeConstants.kSpinningIntakeMotorID);
        m_spinningIntakeMotor.configFactoryDefault();
        m_spinningIntakeMotor.setInverted(IntakeConstants.kSpinningIntakeMotorInverted);
    }

    public void runSpinner() {
        m_spinningIntakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakeSpinningMotorForward);
    }

    public void stop() {
        m_spinningIntakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakeSpinningMotorOff);
    }
}