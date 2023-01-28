package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.RelativeEncoder;
import frc.lib.util.CANSparkMaxUtil;
import com.revrobotics.CANSparkMax.ControlType;
import frc.lib.util.CANSparkMaxUtil.Usage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    
    private CANSparkMax angleMotor;
    private final SparkMaxPIDController angleController;
    private RelativeEncoder integratedAngleEncoder;
    public Pivot(){//intialization method
        angleMotor = new CANSparkMax(40,MotorType.kBrushless);//need to move constant
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(1);//set current limit in amps
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);//set idlemode to brake, can be kCoast
        integratedAngleEncoder.setPositionConversionFactor(1);//ticks to rotations
       // angleController.setP(Constants.Swerve.angleKP);
       // angleController.setI(Constants.Swerve.angleKI);
        //angleController.setD(Constants.Swerve.angleKD);
       // angleController.setFF(Constants.Swerve.angleKFF);
        angleMotor.enableVoltageCompensation(1.0);//voltage compensation
        angleMotor.burnFlash();
        integratedAngleEncoder.setPosition(0.0);
      }
    
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    private void setAngle(double desiredAngle){
        angleController.setReference(desiredAngle, ControlType.kPosition);
    }

    public void goTo90(){
        System.out.print("90");
        angleController.setReference(90, ControlType.kPosition);
    }

    public void goTo0(){
        System.out.print("0");
        angleController.setReference(0, ControlType.kPosition);
    }
}
