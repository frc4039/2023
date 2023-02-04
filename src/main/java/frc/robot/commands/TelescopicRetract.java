// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescopic;

public class TelescopicRetract extends CommandBase {
  private final Telescopic m_Telescopic;
  
  /** Creates a new TelescopicRetract. */
  public TelescopicRetract(Telescopic telescopic) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Telescopic = telescopic;
    addRequirements(m_Telescopic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Telescopic.armReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Telescopic.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
