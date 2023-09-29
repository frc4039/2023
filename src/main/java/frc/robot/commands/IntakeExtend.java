// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class IntakeExtend extends CommandBase {
    private final Intake m_Intake;
    private final Pivot m_Pivot;
    private final boolean m_manualExtend;

    public IntakeExtend(Intake intake, Pivot pivot, boolean manualExtend) {
        m_Intake = intake;
        m_Pivot = pivot;
        m_manualExtend = manualExtend;
        addRequirements(m_Intake, m_Pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ShouldExtend()) {
            m_Intake.extend();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.stopIntake();
    }

    private boolean ShouldExtend() {
        return (m_manualExtend || m_Pivot.getEncoder() < -120);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (ShouldExtend()) {
            if (m_Intake.atSetpoint(IntakeConstants.kIntakePositionRightExtended)) {
                return true;
            }

            return false;
        } else {
            return true;
        }
    }
}
