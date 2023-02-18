// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetRobotPose extends InstantCommand {

  private Swerve m_swerve;
  private Pose2d m_pose;
  public ResetRobotPose(Swerve swerve, Pose2d pose) {
    m_swerve = swerve;
    m_pose = pose;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.setPoseEstimator(m_pose);
  }
}
