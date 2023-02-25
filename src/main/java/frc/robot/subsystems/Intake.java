package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public Intake() {
        m_intakeMotorRight = new CANSparkMax(IntakeConstants.kIntakeRightMotorID, MotorType.kBrushless);// need to move
                                                                                                        // constant
        m_intakeMotorRight.restoreFactoryDefaults();
        m_intakeMotorRight.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
        m_integratedIntakeEncoderRight = m_intakeMotorRight.getEncoder();
        m_intakeControllerRight = m_intakeMotorRight.getPIDController();
        m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeLeftMotorID, MotorType.kBrushless);// need to move
                                                                                                      // constant
        m_intakeMotorLeft.restoreFactoryDefaults();
        m_intakeMotorLeft.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
        m_integratedIntakeEncoderLeft = m_intakeMotorLeft.getEncoder();
        m_intakeControllerLeft = m_intakeMotorLeft.getPIDController();

        ConfigIntakeMotor(m_intakeMotorRight, IntakeConstants.kIntakeRightMotorInverted, m_integratedIntakeEncoderRight,
                m_intakeControllerRight);
        ConfigIntakeMotor(m_intakeMotorLeft, IntakeConstants.kIntakeLeftMotorInverted, m_integratedIntakeEncoderLeft,
                m_intakeControllerLeft);

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addNumber("Intake left", () -> GetIntakePositionLeft());
        tab.addNumber("Intake right", () -> GetIntakePositionRight());
    }

    public double GetIntakePositionRight() {
        return m_integratedIntakeEncoderRight.getPosition();
    }

    public double GetIntakePositionLeft() {
        return m_integratedIntakeEncoderLeft.getPosition();
    }

    public void goToPosition(double positionRight, double positionLeft) {
        m_intakeControllerRight.setReference(positionRight, ControlType.kSmartMotion);
        m_intakeControllerLeft.setReference(positionLeft, ControlType.kSmartMotion);
    }

    public boolean atSetpoint(double position) {
        return (Math.abs(position
                - GetIntakePositionRight()) <= IntakeConstants.kIntakeAllowableError);
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

    public void stopIntake() {
        m_intakeMotorRight.set(IntakeConstants.kStoppedSpeed);
        m_intakeMotorLeft.set(IntakeConstants.kStoppedSpeed);
    }

    @Override
    public void periodic() {}

    private void ConfigIntakeMotor(CANSparkMax motor, boolean invertedMode, RelativeEncoder integratedEncoder,
            SparkMaxPIDController controller) {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kPositionOnly);
        motor.setInverted(invertedMode);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);// set idlemode to brake, can be kCoast
        integratedEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);// ticks to rotations
        controller.setP(IntakeConstants.kIntakeKP);
        controller.setI(IntakeConstants.kIntakeKI);
        controller.setD(IntakeConstants.kIntakeKD);
        controller.setFF(IntakeConstants.kIntakeKFF);
        controller.setSmartMotionMaxVelocity(IntakeConstants.kSmartMotionMaxVelocity, IntakeConstants.kSlotId);
        controller.setSmartMotionMinOutputVelocity(IntakeConstants.kSmartMotionMinOutputVelocity,
                IntakeConstants.kSlotId);
        controller.setSmartMotionMaxAccel(IntakeConstants.kSmartMotionMaxAccel, IntakeConstants.kSlotId);
        controller.setSmartMotionAllowedClosedLoopError(IntakeConstants.kIntakeAllowableError, IntakeConstants.kSlotId);
        motor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);// voltage compensation
        motor.burnFlash();
        integratedEncoder.setPosition(IntakeConstants.kIntakePositionRightRetracted);
    }
}