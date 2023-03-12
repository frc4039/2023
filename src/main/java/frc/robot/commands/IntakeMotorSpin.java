// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSpinner;

public class IntakeMotorSpin extends CommandBase {
    private final IntakeSpinner m_intakeSpinner;

    public IntakeMotorSpin(IntakeSpinner intakeSpinner) {
        m_intakeSpinner = intakeSpinner;
        addRequirements(m_intakeSpinner);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intakeSpinner.runSpinner();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intakeSpinner.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
