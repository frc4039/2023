package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.PivotConstants;
import frc.robot.Robot;

public class Pivot extends SubsystemBase {

    private CANSparkMax m_pivotLeftMotor;
    private CANSparkMax m_pivotRightMotor;
    private DutyCycleEncoder m_pivotEncoder;

    private PIDController controller = new PIDController((0.15 * 0.5), 0, 0);
    private State goalState = new State(PivotConstants.kPositionTravel, 0);
    private State currentState = new State(PivotConstants.kPositionTravel, 0);
    private boolean pidRunning = false;

    public Pivot() {
        ConfigMotor(m_pivotLeftMotor, PivotConstants.kPivotLeftMotorID, false);
        ConfigMotor(m_pivotRightMotor, PivotConstants.kPivotRightMotorID, true);
        m_pivotRightMotor.follow(m_pivotLeftMotor);

        m_pivotEncoder = new DutyCycleEncoder(PivotConstants.kEncoderChannel);

        controller.setTolerance(PivotConstants.kPivotAllowableError);

        // Create a shuffleboard tab for this subsystem and add relevant data.
        ShuffleboardTab tab = Shuffleboard.getTab("Pivot");
        tab.addDouble("Absolute Encoder", () -> getEncoder());
        tab.addDouble("Absoulte Encoder (raw)", () -> m_pivotEncoder.getAbsolutePosition());
        tab.addDouble("Target Position", () -> goalState.position);
        tab.addDouble("Pivot Motor Output", () -> m_pivotLeftMotor.getAppliedOutput());
    }

    /**
     * Disable the PID control for the pivot.
     */
    public void stop() {
        pidRunning = false;
        currentState.velocity = 0;
    }

    /**
     * Enable the PID for the pivot, and set the target to the setpoint.
     * 
     * @param setpoint Desired setpoint. Given in degrees from vertical.
     */
    public void setSetpoint(double setpoint) {
        pidRunning = true;

        currentState.velocity = 0;
        currentState.position = getEncoder();
        goalState.position = setpoint;
        goalState.velocity = 0;
    }

    /**
     * Returns the pivot's target setpoint. Given in degrees from vertical.
     */
    public double getSetpoint() {
        return goalState.position;
    }

    /**
     * Returns true when the Pivot has reached its setpoint.
     */
    public boolean atSetpoint() {
        return Math.abs(currentState.position - goalState.position) < controller.getPositionTolerance()
                && Math.abs(currentState.velocity - goalState.velocity) < controller.getVelocityTolerance();
    }

    /**
     * Returns true when the Pivot has reached the position passed as a parameter.
     */
    public boolean atSetpoint(double position) {
        return Math.abs(currentState.position - position) < controller.getPositionTolerance()
                && Math.abs(currentState.velocity - goalState.velocity) < controller.getVelocityTolerance();
    }

    /**
     * Returns the current encoder reading. Given in degrees from vertical.
     */
    public double getEncoder() {
        return (360.0 * m_pivotEncoder.getAbsolutePosition()) + PivotConstants.kPivotVerticalOffset;
    }

    private void ConfigMotor(CANSparkMax pivotMotor, int motorId, boolean isInverted) {
        pivotMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor, Usage.kPositionOnly);
        pivotMotor.setInverted(isInverted);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);// set idlemode to brake, can be kCoast
        pivotMotor.enableVoltageCompensation(PivotConstants.kNominalVoltage);// voltage compensation
        pivotMotor.setSmartCurrentLimit(PivotConstants.kSmartCurrentLimit);
    }

    /**
     * Runs periodically (every 20 ms). Updates the motion profile, FF, and PID.
     */
    @Override
    public void periodic() {
        if (pidRunning) {
            // Calculate the desired position and velocity to follow a trapezoidal
            // motion profile. A trapezoidal motion profile will ramp up, reach a
            // travelling speed, and then ramp down. This limits accelleration and
            // max speed during the motion.
            TrapezoidProfile profile = new TrapezoidProfile(PivotConstants.kProfileConstraints,
                    goalState,
                    currentState);
            currentState = profile.calculate(Robot.kDefaultPeriod);

            // The PID will correct for errors in the feed-forward's prediction.
            double pidVolts = controller.calculate(getEncoder(), currentState.position);

            m_pivotLeftMotor.setVoltage(pidVolts);
        } else {
            // If the PID is disabled, stop outputting to the motor.
            m_pivotLeftMotor.stopMotor();
        }
    }
}
