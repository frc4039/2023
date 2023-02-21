package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax m_pivotMotor;
    private DutyCycleEncoder m_pivotEncoder;
    private PIDController controller;

    public Pivot(){
        m_pivotMotor = new CANSparkMax(PivotConstants.kPivotMotorID, MotorType.kBrushless);
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor.setSmartCurrentLimit(PivotConstants.kSmartCurrentLimit);
        m_pivotEncoder = new DutyCycleEncoder(PivotConstants.kEncoderChannel);
        configPivotMotor();
        controller = new PIDController(2.0, 0, 0);
        controller.setSetpoint(PivotConstants.kPositionTravel);
    }

    public void stop(){
        controller.setSetpoint(GetPivotPosition());
    }

    public double GetPivotPosition(){
        return m_pivotEncoder.getAbsolutePosition();
    }

    public double GetTargetPosition() {
        return controller.getSetpoint();
    }

    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }
    
    @Override
    public void periodic(){
        m_pivotMotor.set(controller.calculate(GetPivotPosition()));
        SmartDashboard.putNumber("pivot encoder", GetPivotPosition());
        SmartDashboard.putNumber("target pivot position", controller.getSetpoint());
        SmartDashboard.putNumber("pivot output current", m_pivotMotor.getOutputCurrent());
    }

    private void configPivotMotor() {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_pivotMotor, Usage.kPositionOnly);
        m_pivotMotor.setInverted(false);
        m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);//set idlemode to brake, can be kCoast
        m_pivotMotor.enableVoltageCompensation(PivotConstants.kNominalVoltage);//voltage compensation
    }
}
