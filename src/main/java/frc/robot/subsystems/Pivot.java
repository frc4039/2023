package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.geometry.Rotation2d;
//import com.revrobotics.RelativeEncoder;
import frc.lib.util.CANSparkMaxUtil;
import com.revrobotics.CANSparkMax.ControlType;
import frc.lib.util.CANSparkMaxUtil.Usage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax m_pivotMotor;
    private final SparkMaxPIDController m_pivotController;
    private RelativeEncoder m_integratedPivotEncoder;
    private double dashboardTargetPivotPositionValue = 0.0;

    public Pivot(){//intialization method
        m_pivotMotor = new CANSparkMax(Constants.PivotConstants.pivotMotorID, MotorType.kBrushless);//need to move constant
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor.setSmartCurrentLimit(Constants.PivotConstants.smartCurrentLimit);
        m_integratedPivotEncoder = m_pivotMotor.getEncoder();
        m_integratedPivotEncoder.setPosition(0.0);
        m_pivotController = m_pivotMotor.getPIDController();
        configPivotMotor();
    }

    /*public void goToHorizontal(){
        goToPosition(Constants.PivotConstants.positionHorizontal);
    }*/
    
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
        dashboardTargetPivotPositionValue = position;
        m_pivotController.setReference(position, ControlType.kPosition);
    }

    public void setSpeed(double speed){
        m_pivotMotor.set(speed);
    }

    public void moveForward(){
        m_pivotMotor.set(Constants.PivotConstants.speedForward);
    }

    public void moveBack(){
        m_pivotMotor.set(Constants.PivotConstants.speedBack);
    }

    public void stop(){
        m_pivotMotor.set(Constants.PivotConstants.speedStop);
    }

    public void setZero(){
        m_integratedPivotEncoder.setPosition(0.0);
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("pivot encoder", m_integratedPivotEncoder.getPosition());
        SmartDashboard.putNumber("target pivot position", dashboardTargetPivotPositionValue);
        SmartDashboard.putNumber("pivot output current", m_pivotMotor.getOutputCurrent());
    }

    private void configPivotMotor() {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_pivotMotor, Usage.kPositionOnly);
        m_pivotMotor.setInverted(false);
        m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);//set idlemode to brake, can be kCoast
        m_integratedPivotEncoder.setPositionConversionFactor(1);//ticks to rotations
        m_pivotController.setP(Constants.PivotConstants.pivotKP);
        m_pivotController.setI(Constants.PivotConstants.pivotKI);
        m_pivotController.setD(Constants.PivotConstants.pivotKD);
        m_pivotController.setFF(Constants.PivotConstants.pivotKFF);
        m_pivotMotor.enableVoltageCompensation(12.0);//voltage compensation
        m_pivotMotor.burnFlash();
        m_integratedPivotEncoder.setPosition(0.0);
      }
    
    
    /*private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

   private void setAngle(double desiredAngle){
        angleController.setReference(desiredAngle, ControlType.kPosition);
    }*/

}
