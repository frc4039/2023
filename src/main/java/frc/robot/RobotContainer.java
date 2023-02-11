// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
   
     
     
 
/* Operator Buttons */
  private final JoystickButton yOperatorButton =
          new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton aOperatorButton =
      new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton backButton = 
      new JoystickButton(operator, XboxController.Button.kBack.value);
  private final JoystickButton xOperatorButton = 
      new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton bOperatorButton = 
      new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton operatorLeftBumperButton = 
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton operatorRightBumperButton = 
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);
private final JoystickButton driverLeftBumperButton = 
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final Trigger operatorUpButton = new Trigger(() -> operator.getPOV() == 0);
  private final Trigger operatorDownButton = new Trigger(() -> operator.getPOV() == 180);
  private final Trigger operatorLeftButton = new Trigger(() -> operator.getPOV() == 270);
     
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Pivot s_Pivot = new Pivot();
  private final Gripper s_Gripper = new Gripper();
  private final Telescopic s_Telescopic = new Telescopic();
  private final Intake s_Intake = new Intake();
  private final PowerDistributionHub s_PowerDistributionHub = new PowerDistributionHub();

  public class setDefaultCommand{}

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationXAxis),
            () -> -driver.getRawAxis(rotationYAxis),
            () -> zeroGyro.getAsBoolean()));

    CommandScheduler.getInstance().registerSubsystem(s_Pivot);
    CommandScheduler.getInstance().registerSubsystem(s_Telescopic);
    CommandScheduler.getInstance().registerSubsystem(s_PowerDistributionHub);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.resetPoseAndGyro()));

    /* operator Buttons */
    backButton.onTrue(new InstantCommand(()-> s_Pivot.setZero()));
    driverLeftBumperButton.whileTrue(new InstantCommand(()-> s_Telescopic.zeroEncoder()));
    SmartDashboard.putData(new InstantCommand(()-> s_Pivot.setZero()).ignoringDisable(true).withName("Pivot set to zero"));
    operatorLeftBumperButton.onTrue(new GripperRelease(s_Gripper));
    operatorRightBumperButton.onTrue(new GripperRetrieve(s_Gripper));
    yOperatorButton.onTrue(new TravelPosition(s_Telescopic, s_Pivot)); //new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.positionTravel));
    xOperatorButton.onTrue(new CubePickupPosition(s_Telescopic, s_Pivot)); //new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.positionPickupCube));
    aOperatorButton.onTrue(new ConePickupPosition(s_Telescopic, s_Pivot)); //new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.positionPickupCone));
    bOperatorButton.onTrue(new ConeScoringPosition(s_Telescopic, s_Pivot)); //new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.positionScoringCone));
    operatorUpButton.onTrue(new IntakeExtend(s_Intake).withTimeout(IntakeConstants.kIntakeExtendTimeout));
    operatorDownButton.onTrue(new IntakeRetract(s_Intake).withTimeout(IntakeConstants.kIntakeRetractTimeout));
    operatorLeftButton.whileTrue(new IntakeSpin(s_Intake));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 // public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous // RE-ENABLE WHEN AUTO BECOMES AVAILABLE
   // return new exampleAuto(s_Swerve);
 // }
}