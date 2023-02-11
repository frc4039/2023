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

    PIDController m_controller;

    public Telescopic() {
        m_Falcon = new TalonFX(Constants.TelescopicConstants.telescopicMotorID);
        m_Falcon.configFactoryDefault();

        m_FalconConfig = new TalonFXConfiguration();
        m_FalconConfig.supplyCurrLimit.enable = false; // TODO: talk to the electrical subteam to see if this is really needed
        m_FalconConfig.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        m_FalconConfig.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        m_FalconConfig.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is
                                                          // triggered
        m_Falcon.configAllSettings(m_FalconConfig);

        m_controller = new PIDController(TelescopicConstants.telescopicKP, 0, 0);
  }
        
        public void armForward(){
            m_Falcon.set(ControlMode.Position, TelescopicConstants.kTelescopicForward);
        }

        public void armReverse(){
            m_Falcon.set(ControlMode.Position, TelescopicConstants.kTelescopicBack);
        }

        public void armStop() {
            m_Falcon.set(ControlMode.PercentOutput, 0);
        }

        public void zeroEncoder(){
            m_Falcon.setSelectedSensorPosition(0);
        }
                    

        public void setSetPoint(double setpoint) {
            m_controller.setSetpoint(setpoint);
        }

        @Override
        public void periodic(){
            SmartDashboard.putNumber("Telescopic Encoder Value", m_Falcon.getSelectedSensorPosition());
            m_Falcon.set(TalonFXControlMode.PercentOutput,
                    m_controller.calculate(m_Falcon.getSelectedSensorPosition()));
        }
}

// full extension: -30 (still might need calibration as of Feb 10/23)
// full retract: 294 (still might need calibration as of Feb 10/23)
