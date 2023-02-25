// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationXAxis = XboxController.Axis.kRightX.value;
    private final int rotationYAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton driverYButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverLeftBumper = new JoystickButton(driver,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton driverBackButton = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* Operator Buttons */
    private final JoystickButton operatorYButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorAButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorXButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorBButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operator,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRightBumper = new JoystickButton(operator,
            XboxController.Button.kRightBumper.value);
    private final Trigger operatorUpButton = new Trigger(() -> operator.getPOV() == 0);
    private final Trigger operatorDownButton = new Trigger(() -> operator.getPOV() == 180);
    private final Trigger operatorLeftButton = new Trigger(() -> operator.getPOV() == 270);
    private final JoystickButton operatorBackButton = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton operatorStartButton = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final Trigger operatorRightTriggerDepressed = new Trigger(
            () -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > .1);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Gripper s_Gripper = new Gripper();
    private final Telescopic s_Telescopic = new Telescopic();
    private final Intake s_Intake = new Intake();
    private final IntakeSpinner s_IntakeSpinner = new IntakeSpinner();
    private final ConeGuide s_ConeGuide = new ConeGuide();
    private final PowerDistributionHub s_PowerDistributionHub = new PowerDistributionHub();

    public class setDefaultCommand {
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationXAxis),
                        () -> -driver.getRawAxis(rotationYAxis),
                        () -> driverBackButton.getAsBoolean()));

        CommandScheduler.getInstance().registerSubsystem(s_Pivot);
        CommandScheduler.getInstance().registerSubsystem(s_Telescopic);
        CommandScheduler.getInstance().registerSubsystem(s_Gripper);
        CommandScheduler.getInstance().registerSubsystem(s_Intake);
        CommandScheduler.getInstance().registerSubsystem(s_IntakeSpinner);
        CommandScheduler.getInstance().registerSubsystem(s_ConeGuide);
        CommandScheduler.getInstance().registerSubsystem(s_PowerDistributionHub);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        driverBackButton.onTrue(new InstantCommand(() -> s_Swerve.resetPoseAndGyro()));
        driverLeftBumper.onTrue(new InstantCommand(() -> s_Telescopic.zeroEncoder()));
        driverRightBumper
                .onTrue(new SequentialCommandGroup(new Command[] {
                        new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.kPositionScoringConeRelease),
                        new GripperRelease(s_Gripper).withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                        new ParallelCommandGroup(new Command[] {
                                new TelescopicRetract(s_Telescopic),
                                new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.kPositionTravel) }) }));
        driverYButton
            .whileTrue(new TeleopSwerveAtFixedRotation(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                0));

        /* Operator Buttons */
        operatorLeftBumper.onTrue(new GripperRelease(s_Gripper));
        operatorRightBumper.onTrue(new GripperRetrieve(s_Gripper));
        operatorYButton.onTrue(new SeqCmdTravelPosition(s_Telescopic, s_ConeGuide, s_Pivot, s_Intake));
        operatorXButton.onTrue(new SeqCmdCubePickupPosition(s_Telescopic, s_ConeGuide, s_Gripper, s_Intake, s_Pivot));
        operatorBButton.onTrue(new SeqCmdConePickupPosition(s_Telescopic, s_Gripper, s_ConeGuide, s_Pivot, s_Intake));
        operatorAButton.onTrue(new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.kPositionScoringCone));
        operatorUpButton.whileTrue(new IntakeMotorSpin(s_IntakeSpinner));

        operatorBackButton.onTrue(new TelescopicScoringExtendMid(s_Telescopic, s_Pivot));
        operatorStartButton.onTrue(new TelescopicScoringExtendFar(s_Telescopic, s_Pivot));
    }

    public Swerve getSwerve() {
        return s_Swerve;
    }

    public Gripper getGripper() {
        return s_Gripper;
    }

    public Telescopic getTelescopic() {
        return s_Telescopic;
    }

    public Pivot getPivot() {
        return s_Pivot;
    }

    public ConeGuide getConeGuide() {
        return s_ConeGuide;
    }

    public Intake getIntake() {
        return s_Intake;
    }

    public Command getAutonomousCommand() {
        Autos autos = new Autos(this);
        return autos.dropNDrive();
    }
}