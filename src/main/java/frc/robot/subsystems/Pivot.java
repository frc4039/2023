package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;
import com.revrobotics.CANSparkMax.ControlType;
import frc.lib.util.CANSparkMaxUtil.Usage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.RelativeEncoder;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax m_pivotMotor;
    private final SparkMaxPIDController m_pivotController;
    //private RelativeEncoder m_integratedPivotEncoder;
    private DutyCycleEncoder m_pivotEncoder;
    private double m_targetPivotPositionValue = 0.0;
    private double pivotAbsolutePosition;

    public Pivot(){//intialization method
        m_pivotMotor = new CANSparkMax(Constants.PivotConstants.pivotMotorID, MotorType.kBrushless);//need to move constant
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor.setSmartCurrentLimit(Constants.PivotConstants.smartCurrentLimit);
        //m_integratedPivotEncoder = m_pivotMotor.getEncoder();
        m_pivotEncoder = new DutyCycleEncoder(Constants.PivotConstants.encoderChannel);
        //m_integratedPivotEncoder.setPosition(0.0);
        m_pivotController = m_pivotMotor.getPIDController();
        configPivotMotor();
    }
    
    public void goToPickup(){
        goToPosition(Constants.PivotConstants.positionPickupCone);
    }

    public void goToScoring(){
        goToPosition(Constants.PivotConstants.positionScoringCone);
    }

    public void goToTravel(){
        goToPosition(Constants.PivotConstants.positionTravel);
    }

    public void goToPosition(double position){
        m_targetPivotPositionValue = position;
        m_pivotController.setReference(position, ControlType.kPosition);
    }

    public void moveForward(){
        m_pivotMotor.set(Constants.PivotConstants.speedForward);
    }

    public void runAtSpeed(double speed) {
        m_pivotMotor.set(speed);
    }

    public void moveBack(){
        m_pivotMotor.set(Constants.PivotConstants.speedBack);
    }

    public void stop(){
        m_pivotMotor.set(Constants.PivotConstants.speedStop);
    }

    public void setZero(){
        m_pivotEncoder.reset();
    }

    public double GetPivotPosition(){
        return m_pivotEncoder.getAbsolutePosition();
    }

    public double GetTargetPosition() {
        return m_targetPivotPositionValue;
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("pivot encoder", GetPivotPosition());
        SmartDashboard.putNumber("target pivot position", m_targetPivotPositionValue);
        SmartDashboard.putNumber("pivot output current", m_pivotMotor.getOutputCurrent());
    }

    private void configPivotMotor() {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_pivotMotor, Usage.kPositionOnly);
        m_pivotMotor.setInverted(false);
        m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);//set idlemode to brake, can be kCoast
        m_pivotController.setP(Constants.PivotConstants.pivotKP);
        m_pivotController.setI(Constants.PivotConstants.pivotKI);
        m_pivotController.setD(Constants.PivotConstants.pivotKD);
        m_pivotController.setFF(Constants.PivotConstants.pivotKFF);
        m_pivotMotor.enableVoltageCompensation(12.0);//voltage compensation
        m_pivotMotor.burnFlash();
    }
}
