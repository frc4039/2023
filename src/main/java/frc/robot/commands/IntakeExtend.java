// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeExtend extends CommandBase {
  private final Intake m_Intake;

  /** Creates a new IntakeDeploy. */
  public IntakeExtend(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(IntakeConstants.kIntakeExtended - m_Intake.GetIntakePosition()) <= IntakeConstants.intakeAllowableError)
    {
      return true;
    }

    return false;
  }
}
