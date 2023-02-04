// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final JoystickButton yButton =
      new JoystickButton(operator, XboxController.Button.kY.value);
 // private final Button upDPad = operator.getPOV
  private final JoystickButton aButton =
      new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton bButton =
      new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton xButton =
      new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton backButton = 
      new JoystickButton(operator, XboxController.Button.kBack.value);
  private final JoystickButton xDriverButton = 
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton bDriverButton = 
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton operatorLeftBumperButton = 
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton operatorRightBumperButton = 
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);

     
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Pivot s_Pivot = new Pivot();
  private final Gripper s_Gripper = new Gripper();
  private final Telescopic s_Telescopic = new Telescopic();

  public class setDefaultCommand{}

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationXAxis),
            () -> -driver.getRawAxis(rotationYAxis)));

    CommandScheduler.getInstance().registerSubsystem(s_Pivot);
    CommandScheduler.getInstance().registerSubsystem(s_Telescopic);

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
    //yButton.onTrue(new InstantCommand(() -> s_Pivot.goToTravel()));
    //aButton.onTrue(new InstantCommand(() -> s_Pivot.goToPickup()));
    //bButton.onTrue(new InstantCommand(() -> s_Pivot.goToHorizontal()));
    //xButton.onTrue(new InstantCommand(() -> s_Pivot.goToScoring()));
    backButton.onTrue(new InstantCommand(()-> s_Pivot.setZero()));
    xDriverButton.whileTrue(new GripperRelease(s_Gripper));
    bDriverButton.onTrue(new GripperRetrieve(s_Gripper));
    yButton.whileTrue(new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.speedForward));
    aButton.whileTrue(new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.speedBack));
    operatorLeftBumperButton.whileTrue(new TelescopicRetract(s_Telescopic));
    operatorRightBumperButton.whileTrue(new TelescopicExtend(s_Telescopic));
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