// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class PivotMoveToPosition extends CommandBase {
  /** Creates a new PivotMoveToPosition. */
  Pivot m_pivot;
  double m_position;
  public PivotMoveToPosition(Pivot pivot, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
    m_position = position;
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.goToPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
