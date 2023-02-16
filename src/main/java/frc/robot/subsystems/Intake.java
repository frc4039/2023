package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private CANSparkMax m_intakeMotor1;
    private final SparkMaxPIDController m_intakeController1;
    private RelativeEncoder m_integratedIntakeEncoder1;
    private CANSparkMax m_intakeMotor2;
    //private final SparkMaxPIDController m_intakeController2;
    private RelativeEncoder m_integratedIntakeEncoder2;
    //private final VictorSPX m_intakeMotor1;
    //private final VictorSPX m_intakeMotor2;
    private final TalonSRX m_spinningIntakeMotor;

    public Intake() {
        m_intakeMotor1 = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorID1, MotorType.kBrushless);//need to move constant
        m_intakeMotor1.restoreFactoryDefaults();
        m_intakeMotor1.setSmartCurrentLimit(Constants.IntakeConstants.smartCurrentLimit);
        m_integratedIntakeEncoder1 = m_intakeMotor1.getEncoder();
        m_integratedIntakeEncoder1.setPosition(0.0);
        m_intakeController1 = m_intakeMotor1.getPIDController();
        m_intakeMotor2 = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorID2, MotorType.kBrushless);//need to move constant
        m_intakeMotor2.restoreFactoryDefaults();
        m_intakeMotor2.setSmartCurrentLimit(Constants.IntakeConstants.smartCurrentLimit);
        m_integratedIntakeEncoder2 = m_intakeMotor2.getEncoder();
        m_integratedIntakeEncoder2.setPosition(0.0);
        //m_intakeController2 = m_intakeMotor2.getPIDController();
        m_intakeMotor2.follow(m_intakeMotor1);

        /*m_intakeMotor1 = new VictorSPX(IntakeConstants.kIntakeMotor1);
        m_intakeMotor2 = new VictorSPX(IntakeConstants.kIntakeMotor2);
        m_intakeMotor1.configFactoryDefault();
        m_intakeMotor2.configFactoryDefault();
        m_intakeMotor1.setInverted(IntakeConstants.kIntakeMotor1Inverted);
        m_intakeMotor2.setInverted(IntakeConstants.kIntakeMotor2Inverted);
        m_intakeMotor1.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor2.setNeutralMode(NeutralMode.Brake);*/

        m_spinningIntakeMotor = new TalonSRX(IntakeConstants.spinningIntakeMotorID);
        m_spinningIntakeMotor.configFactoryDefault();
        m_spinningIntakeMotor.setInverted(IntakeConstants.kSpinningIntakeMotorInverted);
    }

    public double GetIntakePosition(){
        return m_integratedIntakeEncoder1.getPosition();
    }

    public void goToPosition(double position){
        m_intakeController1.setReference(position, ControlType.kPosition);
    }

    public void extend() {
        //m_intakeMotor1.set(IntakeConstants.kIntakeMotorPercentExtend);
        goToPosition(IntakeConstants.kIntakeExtended);
    }

    public void retract() {
       // m_intakeMotor1.set(IntakeConstants.kIntakeMotorPercentRetract);
       goToPosition(IntakeConstants.kIntakeRetracted);
    }

    public void pickup() {
        goToPosition(IntakeConstants.kIntakePickup);
     }
 
    public void setSpinningMotorOn() {
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.intakeSpinningMotorForward);
    }

    public void setSpinningMotorOff() {
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.intakeSpinningMotorOff);
    }

    public void stopIntake() {
        m_intakeMotor1.set(0);
    }

    public void stop() {
        m_intakeMotor1.set(0);
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake encoder", GetIntakePosition());
    }
}