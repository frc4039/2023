package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.RelativeEncoder;
import frc.lib.util.CANSparkMaxUtil;
import com.revrobotics.CANSparkMax.ControlType;
import frc.lib.util.CANSparkMaxUtil.Usage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.TelescopicConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescopic extends SubsystemBase {
    private TalonFX m_Falcon; // creates a new TalonFX, ID can be found in Line 190(ish) of Constants.java
    TalonFXConfiguration m_FalconConfig;

    public Telescopic() {
        m_Falcon = new TalonFX(Constants.TelescopicConstants.telescopicMotorID);
        m_Falcon.configFactoryDefault();

        m_FalconConfig = new TalonFXConfiguration();
        m_FalconConfig.supplyCurrLimit.enable = false; // TODO: talk to the electrical subteam to see if this is really needed
        m_FalconConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        m_FalconConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        m_FalconConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        m_FalconConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

      //  m_FalconConfig.peakOutputForward = 0.1;
      //  m_FalconConfig.peakOutputReverse = -0.1;
        m_Falcon.configAllSettings(m_FalconConfig);
      //  m_Falcon.configNominalOutputForward(0,0);
      //  m_Falcon.configNominalOutputReverse(0,0);
        m_Falcon.configPeakOutputForward(12, 0);
        m_Falcon.configPeakOutputReverse(-12, 0);
        m_Falcon.config_kP(0, TelescopicConstants.telescopicKP, 0);
        m_Falcon.config_kI(0, TelescopicConstants.telescopicKI, 0);
        m_Falcon.config_kD(0, TelescopicConstants.telescopicKD, 0);
        m_Falcon.config_kF(0, TelescopicConstants.telescopicKFF, 0);
        m_Falcon.configClosedLoopPeakOutput(0, 0.2);
        m_Falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_Falcon.setNeutralMode(NeutralMode.Brake);
       // m_Falcon.configAllowableClosedloopError(0, 700, 0);
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
            SmartDashboard.putNumber("Closed Loop Error", m_Falcon.getClosedLoopError(0));
        }
}

// full extension: 52300
// mid: 28748
// full retract: 5 (it is actually 2, but to be safe, it's 5 right now)
