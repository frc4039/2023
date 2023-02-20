package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TelescopicConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescopic extends SubsystemBase {
    private TalonFX m_Falcon;
    private TalonFXConfiguration m_FalconConfig;

    public Telescopic() {
        m_Falcon = new TalonFX(TelescopicConstants.kTelescopicMotorID);
        m_Falcon.configFactoryDefault();

        m_FalconConfig = new TalonFXConfiguration();
        m_FalconConfig.supplyCurrLimit.enable = false; // TODO: talk to the electrical subteam to see if this is really needed
        m_FalconConfig.supplyCurrLimit.triggerThresholdCurrent = TelescopicConstants.kTriggerThresholdCurrent; // the peak supply current, in amps
        m_FalconConfig.supplyCurrLimit.triggerThresholdTime = TelescopicConstants.kTriggerThresholdTime; // the time at the peak supply current before the limit triggers, in sec
        m_FalconConfig.supplyCurrLimit.currentLimit = TelescopicConstants.kCurrentLimit; // the current to maintain if the peak supply limit is triggered
        m_FalconConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        m_Falcon.configAllSettings(m_FalconConfig);
        m_Falcon.configPeakOutputForward(TelescopicConstants.kPeakOutputForwardPercent, TelescopicConstants.kTimeoutMs);
        m_Falcon.configPeakOutputReverse(TelescopicConstants.kPeakOutputReversePercent, TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kP(TelescopicConstants.kTelescopicSlotIdxKP, TelescopicConstants.kTelescopicKP, TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kI(TelescopicConstants.kTelescopicSlotIdxKI, TelescopicConstants.kTelescopicKI, TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kD(TelescopicConstants.kTelescopicSlotIdxKD, TelescopicConstants.kTelescopicKD, TelescopicConstants.kTimeoutMs);
        m_Falcon.config_kF(TelescopicConstants.kTelescopicSlotIdxKFF, TelescopicConstants.kTelescopicKFF, TelescopicConstants.kTimeoutMs);
        m_Falcon.configClosedLoopPeakOutput(TelescopicConstants.kClosedLoopPeakOutputSlotIdx, TelescopicConstants.kClosedLoopPeakOutputPercent);
        m_Falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, TelescopicConstants.kSelectedFeedbackSensorPidIdx, TelescopicConstants.kTimeoutMs);
        m_Falcon.setNeutralMode(NeutralMode.Brake);
       m_Falcon.configAllowableClosedloopError(TelescopicConstants.kAllowableClosedloopErrorSlotIdx, TelescopicConstants.kAllowableClosedloopError, TelescopicConstants.kTimeoutMs);
  }

        public void armSetPosition(double position){
            m_Falcon.set(ControlMode.Position, position);
        }

        public double getEncoderPosition(){
            return m_Falcon.getSelectedSensorPosition();
        }

        public void armStop() {
            m_Falcon.set(ControlMode.PercentOutput, 0);
        }

        public void zeroEncoder(){
            m_Falcon.setSelectedSensorPosition(0);
        }

        @Override
        public void periodic(){
            SmartDashboard.putNumber("Telescopic", m_Falcon.getSelectedSensorPosition());
            SmartDashboard.putNumber("Closed Loop Error", m_Falcon.getClosedLoopError(TelescopicConstants.kPrimaryClosedLoopErrorPidx));
        }
}
