// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;

public class GripperRelease extends CommandBase {
  private final Gripper m_Gripper;
  private final Pivot m_Pivot;
  
  /** Creates a new GripperRelease. */
  public GripperRelease(Gripper gripper, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Gripper = gripper;
    m_Pivot = pivot;
    addRequirements(m_Gripper, m_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Pivot.GetPivotPosition() == PivotConstants.positionScoringCone) {
      m_Pivot.goToPosition(-19.5);
    }
    m_Gripper.setOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Gripper.setOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
