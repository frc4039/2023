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

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private CANSparkMax m_intakeMotorRight;
    private final SparkMaxPIDController m_intakeControllerRight;
    private RelativeEncoder m_integratedIntakeEncoderRight;
    private CANSparkMax m_intakeMotorLeft;
    private final SparkMaxPIDController m_intakeControllerLeft;
    private RelativeEncoder m_integratedIntakeEncoderLeft;
    private final TalonSRX m_spinningIntakeMotor;

    public Intake() {
        m_intakeMotorRight = new CANSparkMax(IntakeConstants.kIntakeRightMotorID, MotorType.kBrushless);//need to move constant
        m_intakeMotorRight.restoreFactoryDefaults();
        m_intakeMotorRight.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
        m_integratedIntakeEncoderRight = m_intakeMotorRight.getEncoder();
        m_intakeControllerRight = m_intakeMotorRight.getPIDController();
        m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeLeftMotorID, MotorType.kBrushless);//need to move constant
        m_intakeMotorLeft.restoreFactoryDefaults();
        m_intakeMotorLeft.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
        m_integratedIntakeEncoderLeft = m_intakeMotorLeft.getEncoder();
        m_intakeControllerLeft = m_intakeMotorLeft.getPIDController();

        ConfigIntakeMotor(m_intakeMotorRight, IntakeConstants.kIntakeRightMotorInverted, m_integratedIntakeEncoderRight, m_intakeControllerRight);
        ConfigIntakeMotor(m_intakeMotorLeft, IntakeConstants.kIntakeLeftMotorInverted, m_integratedIntakeEncoderLeft, m_intakeControllerLeft);

        m_spinningIntakeMotor = new TalonSRX(IntakeConstants.kSpinningIntakeMotorID);
        m_spinningIntakeMotor.configFactoryDefault();
        m_spinningIntakeMotor.setInverted(IntakeConstants.kSpinningIntakeMotorInverted);
    }

    public double GetIntakePositionRight(){
        return m_integratedIntakeEncoderRight.getPosition();
    }

    public double GetIntakePositionLeft(){
        return m_integratedIntakeEncoderLeft.getPosition();
    }

    public void goToPosition(double positionRight, double positionLeft){
        m_intakeControllerRight.setReference(positionRight, ControlType.kSmartMotion);
        m_intakeControllerLeft.setReference(positionLeft, ControlType.kSmartMotion);
    }

    public void extend() {
        goToPosition(IntakeConstants.kIntakePositionRightExtended, IntakeConstants.kIntakePositionLeftExtended);
    }

    public void retract() {
       goToPosition(IntakeConstants.kIntakePositionRightRetracted, IntakeConstants.kIntakePositionLeftRetracted);
    }

    public void pickup() {
        goToPosition(IntakeConstants.kIntakePositionRightPickup, IntakeConstants.kIntakePositionLeftPickup);
     }
 
    public void setSpinningMotorOn() {
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kIntakeSpinningMotorForward);
    }

    public void setSpinningMotorOff() {
        m_spinningIntakeMotor.set(TalonSRXControlMode.PercentOutput, IntakeConstants.kIntakeSpinningMotorOff);
    }

    public void stopIntake() {
        m_intakeMotorRight.set(IntakeConstants.kStoppedSpeed);
        m_intakeMotorLeft.set(IntakeConstants.kStoppedSpeed);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Right", GetIntakePositionRight());
        SmartDashboard.putNumber("Intake Left", GetIntakePositionLeft());
    }

    private void ConfigIntakeMotor(CANSparkMax motor, boolean invertedMode, RelativeEncoder integratedEncoder, SparkMaxPIDController controller)
    {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kPositionOnly);
        motor.setInverted(invertedMode);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);//set idlemode to brake, can be kCoast
        integratedEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);//ticks to rotations
        controller.setP(IntakeConstants.kIntakeKP);
        controller.setI(IntakeConstants.kIntakeKI);
        controller.setD(IntakeConstants.kIntakeKD);
        controller.setFF(IntakeConstants.kIntakeKFF);
        controller.setSmartMotionMaxVelocity(IntakeConstants.kSmartMotionMaxVelocity, IntakeConstants.kSlotId);
        controller.setSmartMotionMinOutputVelocity(IntakeConstants.kSmartMotionMinOutputVelocity, IntakeConstants.kSlotId);
        controller.setSmartMotionMaxAccel(IntakeConstants.kSmartMotionMaxAccel, IntakeConstants.kSlotId);
        controller.setSmartMotionAllowedClosedLoopError(IntakeConstants.kIntakeAllowableError, IntakeConstants.kSlotId);
        motor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);//voltage compensation
        motor.burnFlash();
        integratedEncoder.setPosition(IntakeConstants.kIntakePositionRightRetracted);
    }
}