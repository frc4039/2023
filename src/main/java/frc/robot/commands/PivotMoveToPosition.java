// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class PivotMoveToPosition extends CommandBase {

    Pivot m_pivot;
    double m_position;

    public PivotMoveToPosition(Pivot pivot, double position) {
        addRequirements(pivot);
        m_pivot = pivot;
        m_position = position;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pivot.setSetpoint(m_position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_pivot.atSetpoint();
    }
}
