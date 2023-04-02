package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopicConstants;

public class Telescopic extends SubsystemBase {
    private TalonFX m_Falcon;
    private TalonFXConfiguration m_FalconConfig;

    public Telescopic() {
        m_Falcon = new TalonFX(TelescopicConstants.kTelescopicMotorID);
        m_Falcon.configFactoryDefault();

        m_FalconConfig = new TalonFXConfiguration();
        m_FalconConfig.supplyCurrLimit.enable = false;
        m_FalconConfig.supplyCurrLimit.triggerThresholdCurrent = TelescopicConstants.kTriggerThresholdCurrent;
        m_FalconConfig.supplyCurrLimit.triggerThresholdTime = TelescopicConstants.kTriggerThresholdTime;
        m_FalconConfig.supplyCurrLimit.currentLimit = TelescopicConstants.kCurrentLimit;
        m_FalconConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        m_Falcon.configAllSettings(m_FalconConfig);
        m_Falcon.configPeakOutputForward(TelescopicConstants.kPeakOutputForwardPercent, TelescopicConstants.kTimeoutMs);
        m_Falcon.configPeakOutputReverse(TelescopicConstants.kPeakOutputReversePercent, TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kP(TelescopicConstants.kTelescopicSlotIdxKP, TelescopicConstants.kTelescopicKP,
                TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kI(TelescopicConstants.kTelescopicSlotIdxKI, TelescopicConstants.kTelescopicKI,
                TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kD(TelescopicConstants.kTelescopicSlotIdxKD, TelescopicConstants.kTelescopicKD,
                TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kF(TelescopicConstants.kTelescopicSlotIdxKFF, TelescopicConstants.kTelescopicKFF,
                TelescopicConstants.kTimeoutMs);
        m_Falcon.configClosedLoopPeakOutput(TelescopicConstants.kClosedLoopPeakOutputSlotIdx,
                TelescopicConstants.kClosedLoopPeakOutputPercent);
        m_Falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                TelescopicConstants.kSelectedFeedbackSensorPidIdx, TelescopicConstants.kTimeoutMs);
        m_Falcon.setNeutralMode(NeutralMode.Brake);
        m_Falcon.configAllowableClosedloopError(TelescopicConstants.kAllowableClosedloopErrorSlotIdx,
                TelescopicConstants.kAllowableClosedloopError, TelescopicConstants.kTimeoutMs);

        ShuffleboardTab tab = Shuffleboard.getTab("Telescopic");
        tab.addNumber("Telescopic", () -> m_Falcon.getSelectedSensorPosition());
        tab.addNumber("Closed Loop Error",
                () -> m_Falcon.getClosedLoopError(TelescopicConstants.kPrimaryClosedLoopErrorPidx));
        tab.addNumber("Current", () -> m_Falcon.getStatorCurrent()).withWidget(BuiltInWidgets.kGraph);
    }

    public void armSetPosition(double position) {
        m_Falcon.set(ControlMode.Position, position);
    }

    public double getEncoderPosition() {
        return m_Falcon.getSelectedSensorPosition();
    }

    public void armStop() {
        m_Falcon.set(ControlMode.PercentOutput, 0);
    }

    public void zeroEncoder() {
        m_Falcon.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
    }
}
