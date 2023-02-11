// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopicConstants;
import frc.robot.subsystems.Telescopic;

public class TelescopicExtend extends CommandBase {
  private final Telescopic m_Telescopic;
  
  /** Creates a new TelescopicExtend. */
  public TelescopicExtend(Telescopic telescopic) {
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
    if (Math.abs(m_Telescopic.getEncoderPosition() - TelescopicConstants.kTelescopicMid) <= 700){
      m_Telescopic.armSetPosition(TelescopicConstants.kTelescopicFar);
    }
    else m_Telescopic.armSetPosition(TelescopicConstants.kTelescopicMid);
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
