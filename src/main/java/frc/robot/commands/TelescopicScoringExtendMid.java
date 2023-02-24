// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TelescopicConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

public class TelescopicScoringExtendMid extends CommandBase {
    private final Telescopic m_Telescopic;
    private final Pivot m_Pivot;
    private double targetPosition;

    /** Creates a new TelescopicExtend. */
    public TelescopicScoringExtendMid(Telescopic telescopic, Pivot pivot) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_Telescopic = telescopic;
        m_Pivot = pivot;
        addRequirements(m_Telescopic);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetPosition = TelescopicConstants.kTelescopicMid;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_Pivot.GetTargetPosition() == PivotConstants.kPositionScoringCone
                || m_Pivot.GetTargetPosition() == PivotConstants.kPositionPickupCube)
            m_Telescopic.armSetPosition(targetPosition);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_Telescopic.getEncoderPosition() - targetPosition) < 700;
    }
}
