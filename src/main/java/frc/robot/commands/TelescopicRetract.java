// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopicConstants;
import frc.robot.subsystems.Telescopic;

public class TelescopicRetract extends CommandBase {
    private final Telescopic m_Telescopic;

    public TelescopicRetract(Telescopic telescopic) {
        m_Telescopic = telescopic;
        addRequirements(m_Telescopic);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Telescopic.armSetPosition(TelescopicConstants.kTelescopicRetracted);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Telescopic.armStop();
        m_Telescopic.zeroEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_Telescopic.getEncoderPosition() - TelescopicConstants.kTelescopicRetracted) < 1700;
    }
}
