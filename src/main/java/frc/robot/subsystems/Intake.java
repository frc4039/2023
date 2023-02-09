package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final VictorSPX m_intakeMotor1;
    private final VictorSPX m_intakeMotor2;
    private final TalonSRX m_spinningIntakeMotor;

    public Intake() {
        m_intakeMotor1 = new VictorSPX(IntakeConstants.kIntakeMotor1);
        m_intakeMotor2 = new VictorSPX(IntakeConstants.kIntakeMotor2);
        m_intakeMotor1.configFactoryDefault();
        m_intakeMotor2.configFactoryDefault();
        m_intakeMotor1.setInverted(IntakeConstants.kIntakeMotor1Inverted);
        m_intakeMotor2.setInverted(IntakeConstants.kIntakeMotor2Inverted);
        m_intakeMotor1.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor2.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor2.follow(m_intakeMotor1);

        m_spinningIntakeMotor = new TalonSRX(IntakeConstants.spinningIntakeMotorID);
        m_spinningIntakeMotor.configFactoryDefault();
        m_spinningIntakeMotor.setInverted(IntakeConstants.kSpinningIntakeMotorInverted);
    }

    public void extend() {
        m_intakeMotor1.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakeMotorPercentExtend);
    }

    public void retract() {
        m_intakeMotor1.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakeMotorPercentRetract);
    }

    public void setSpinningMotorOn() {
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.intakeSpinningMotorForward);
    }

    public void setSpinningMotorOff() {
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.intakeSpinningMotorOff);
    }

    public void stopIntake() {
        m_intakeMotor1.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void stop() {
        m_intakeMotor1.set(VictorSPXControlMode.PercentOutput, 0);
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}