// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConeGuide;

public class ConeGuideDeploy extends CommandBase {
  private final ConeGuide m_ConeGuide;

  /** Creates a new ConeGuideDeploy. */
  public ConeGuideDeploy(ConeGuide coneGuide) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ConeGuide = coneGuide;
    addRequirements(m_ConeGuide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ConeGuide.setForward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ConeGuide.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}