// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeRetract extends CommandBase {
  private final Intake m_Intake;
  private Timer intakeTimer;
  
  /** Creates a new IntakeRetract. */
  public IntakeRetract(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    intakeTimer = new Timer();
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeTimer.get() > 0.5) {
      m_Intake.retract();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(IntakeConstants.kIntakePositionRightRetracted - m_Intake.GetIntakePositionRight()) <= IntakeConstants.kIntakeAllowableError)
    {
      return true;
    }

    return false;
  }
}
