// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.GamePieceSelector.Gamepiece;

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
    private final JoystickButton driverAButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverBButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverXButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton driverBackButton = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final Trigger driverRightTriggerDepressed = new Trigger(
            () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.1);
    private final Trigger driverLeftTriggerDepressed = new Trigger(
            () -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1);
    private final Trigger driverRightButton = new Trigger(() -> driver.getPOV() == 90);
    private final Trigger driverLeftButton = new Trigger(() -> driver.getPOV() == 270);

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
    private final Trigger operatorRightButton = new Trigger(() -> operator.getPOV() == 90);
    private final JoystickButton operatorBackButton = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton operatorStartButton = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final Trigger operatorLeftTriggerDepressed = new Trigger(
            () -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1);
    private final Trigger operatorRightTriggerDepressed = new Trigger(
            () -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.1);
    private final JoystickButton operatorLeftJoystickButton = new JoystickButton(operator,
            XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorRightJoystickButton = new JoystickButton(operator,
            XboxController.Button.kRightStick.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Gripper s_Gripper = new Gripper();
    private final Telescopic s_Telescopic = new Telescopic();
    private final Intake s_Intake = new Intake();
    private final IntakeSpinner s_IntakeSpinner = new IntakeSpinner();
    private final ConeGuide s_ConeGuide = new ConeGuide();
    private final PowerDistributionHub s_PowerDistributionHub = new PowerDistributionHub();
    private final PressureSensor s_PressureSensor = new PressureSensor();
    private final NodeSelector s_NodeSelector = new NodeSelector(this);
    private final BlinkinGamePiece s_BlinkinGamePiece = new BlinkinGamePiece();
    private final GamePieceSelector s_GamePieceSelector = new GamePieceSelector(null);

    private AutoModeSelector autoModeSelector;

    public class setDefaultCommand {
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerveTurn(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationXAxis)));

        autoModeSelector = new AutoModeSelector(this);

        CommandScheduler.getInstance().registerSubsystem(s_Pivot);
        CommandScheduler.getInstance().registerSubsystem(s_Telescopic);
        CommandScheduler.getInstance().registerSubsystem(s_Gripper);
        CommandScheduler.getInstance().registerSubsystem(s_Intake);
        CommandScheduler.getInstance().registerSubsystem(s_IntakeSpinner);
        CommandScheduler.getInstance().registerSubsystem(s_ConeGuide);
        CommandScheduler.getInstance().registerSubsystem(s_PowerDistributionHub);
        CommandScheduler.getInstance().registerSubsystem(s_PressureSensor);
        CommandScheduler.getInstance().registerSubsystem(s_NodeSelector);
        CommandScheduler.getInstance().registerSubsystem(s_BlinkinGamePiece);
        CommandScheduler.getInstance().registerSubsystem(s_GamePieceSelector);

        ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
        Map<String, Object> pressureSensorMax = new HashMap<String, Object>() {
            {
                put("max", 120);
            }
        };
        mainTab.addString("Gamepiece State", () -> s_GamePieceSelector.getCurrentGamepiece().toString()).withSize(1, 1)
                .withPosition(0, 0);
        mainTab.addString("Alliance", () -> DriverStation.getAlliance().toString()).withSize(1, 1)
                .withPosition(1, 0);
        mainTab.addString("Selected Node", () -> s_NodeSelector.getSelectedNodeLabel()).withSize(1, 1)
                .withPosition(2, 0);
        mainTab.addDouble("Gyro", () -> s_Swerve.getYaw().getDegrees()).withSize(1, 1)
                .withPosition(3, 0);
        mainTab.add("AutoMode", autoModeSelector.getAutoChooser()).withSize(2, 1).withPosition(0, 1);
        mainTab.add("Gyro zero", new ZeroGyro(s_Swerve)).withSize(2, 1).withPosition(2, 1);
        mainTab.addDouble("Analog Pressure Sensor", () -> PressureSensor.getAnalogPressureReading()).withSize(2, 1)
                .withPosition(0, 2).withWidget(BuiltInWidgets.kDial).withProperties(pressureSensorMax);
        mainTab.add("Reset angle encoders", new ResetSwerveAngleEncoders(s_Swerve)).withSize(2, 1).withPosition(2, 2);
        mainTab.add("Game Field", s_Swerve.getField()).withSize(5, 3).withPosition(4, 0);

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Drive");
        swerveTab.addBoolean("Scored", driverRightTriggerDepressed).withSize(1, 1).withPosition(0, 1);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        /* ============== */
        driverBackButton.onTrue(new InstantCommand(() -> s_Swerve.resetPoseAndGyro()));

        driverYButton
                .whileTrue(new TeleopSwerveAtFixedRotation(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        0));
        driverAButton
                .whileTrue(new TeleopSwerveAtFixedRotation(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        180));

        // Semi-auto alignment to scoring locations
        driverLeftTriggerDepressed
                .whileTrue(new PIDTranslate(s_Swerve, () -> s_NodeSelector.getSelectedNodeTranslation().getX(),
                        () -> s_NodeSelector.getSelectedNodeTranslation().getY(), () -> 0.0));
        // rotate to HP angle
        driverBButton.whileTrue(new SelectCommand(Map.ofEntries(
                Map.entry(Alliance.Blue,
                        new TeleopSwerveAtFixedRotation(
                                s_Swerve,
                                () -> -driver.getRawAxis(translationAxis),
                                () -> -driver.getRawAxis(strafeAxis),
                                270)),
                Map.entry(Alliance.Red,
                        new TeleopSwerveAtFixedRotation(
                                s_Swerve,
                                () -> -driver.getRawAxis(translationAxis),
                                () -> -driver.getRawAxis(strafeAxis),
                                90)),
                Map.entry(Alliance.Invalid,
                        new InstantCommand())),
                DriverStation::getAlliance));

        // scoring
        driverXButton.onTrue(new CmdGrpScoreLow(s_Pivot, s_Gripper, s_Telescopic, s_BlinkinGamePiece));
        driverRightBumper
                .onTrue(new SelectCommand(
                        Map.ofEntries(
                                Map.entry(Gamepiece.YELLOW,
                                        new CmdGrpScoreYellowMid(s_Pivot, s_Gripper, s_Telescopic,
                                                s_BlinkinGamePiece)),
                                Map.entry(Gamepiece.PURPLE,
                                        new CmdGrpScorePurpleMid(s_Pivot, s_Gripper, s_Telescopic,
                                                s_BlinkinGamePiece))),
                        s_GamePieceSelector::getCurrentGamepiece));
        driverRightTriggerDepressed
                .onTrue(new SelectCommand(
                        Map.ofEntries(
                                Map.entry(Gamepiece.YELLOW,
                                        new CmdGrpScoreYellowHigh(s_Pivot, s_Gripper, s_Telescopic,
                                                s_BlinkinGamePiece)),
                                Map.entry(Gamepiece.PURPLE,
                                        new CmdGrpScorePurpleHigh(s_Pivot, s_Gripper, s_Telescopic,
                                                s_BlinkinGamePiece))),
                        s_GamePieceSelector::getCurrentGamepiece));

        // pivot move to vertical (really only for debugging
        driverRightButton.onTrue(new PivotMoveToPosition(s_Pivot, 0));

        // ====================================================== //

        /* Operator buttons */
        /* ================ */
        // Reset Telescopic encoder - moved here so driver doesnt have to do it!
        operatorLeftJoystickButton.onTrue(new InstantCommand(() -> s_Telescopic.zeroEncoder()));
        // Gripper
        operatorLeftBumper
                .onTrue(new GripperRelease(s_Gripper).withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));
        operatorRightBumper
                .onTrue(new GripperRetrieve(s_Gripper).withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

        // set robot state
        operatorLeftTriggerDepressed.onTrue(new CmdGrpSetPurple(s_GamePieceSelector, s_BlinkinGamePiece));
        operatorRightTriggerDepressed.onTrue(new CmdGrpSetYellow(s_GamePieceSelector, s_BlinkinGamePiece));

        // floor pickup
        operatorXButton.onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(Gamepiece.YELLOW,
                                new CmdGrpConePickupPosition(s_Telescopic, s_Gripper, s_ConeGuide,
                                        s_Pivot, s_Intake)),
                        Map.entry(Gamepiece.PURPLE,
                                new CmdGrpCubePickupPosition(s_Telescopic, s_ConeGuide, s_Gripper,
                                        s_Intake, s_Pivot))),
                s_GamePieceSelector::getCurrentGamepiece));
        // pivot angles
        operatorYButton.onTrue(new CmdGrpTravelPosition(s_Telescopic, s_ConeGuide, s_Pivot, s_Intake));
        operatorAButton.onTrue(new CmdGrpScoringPosition(s_ConeGuide, s_Telescopic, s_Pivot));

        // semi-auto scoring node selector
        operatorUpButton.onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(Gamepiece.YELLOW,
                                new InstantCommand(() -> s_NodeSelector.selectNode(1))),
                        Map.entry(Gamepiece.PURPLE,
                                new InstantCommand(() -> s_NodeSelector.selectNode(7)))),
                s_GamePieceSelector::getCurrentGamepiece));
        operatorLeftButton.onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(Gamepiece.YELLOW,
                                new InstantCommand(() -> s_NodeSelector.selectNode(2))),
                        Map.entry(Gamepiece.PURPLE,
                                new InstantCommand(() -> s_NodeSelector.selectNode(8)))),
                s_GamePieceSelector::getCurrentGamepiece));
        operatorDownButton.onTrue(new SelectCommand(
                Map.ofEntries(
                        Map.entry(Gamepiece.YELLOW,
                                new InstantCommand(() -> s_NodeSelector.selectNode(3))),
                        Map.entry(Gamepiece.PURPLE,
                                new InstantCommand(() -> s_NodeSelector.selectNode(9)))),
                s_GamePieceSelector::getCurrentGamepiece));
        operatorRightButton.onTrue(new InstantCommand(() -> s_NodeSelector.selectNode(4)));
        operatorBackButton.onTrue(new InstantCommand(() -> s_NodeSelector.selectNode(5)));
        operatorStartButton.onTrue(new InstantCommand(() -> s_NodeSelector.selectNode(6)));

        driverLeftButton.onTrue(new SeqCmdCubePickupPosition(s_Telescopic, s_ConeGuide, s_Gripper, s_Intake,
                s_IntakeSpinner, s_Pivot).withTimeout(10));
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

    public IntakeSpinner gIntakeSpinner() {
        return s_IntakeSpinner;
    }

    public AutoModeSelector getAutoModeSelector() {
        return autoModeSelector;
    }

    public Command getAutonomousCommand() {
        return autoModeSelector.getAutoChooser().getSelected();
    }
}