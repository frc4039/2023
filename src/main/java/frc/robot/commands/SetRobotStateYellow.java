// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GamePieceSelector;
import frc.robot.subsystems.GamePieceSelector.Gamepiece;

public class SetRobotStateYellow extends CommandBase {
    private GamePieceSelector m_GamePieceSelector;

    /** Creates a new SetRobotStateYellow. */
    public SetRobotStateYellow(GamePieceSelector gamePieceSelector) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_GamePieceSelector = gamePieceSelector;
        addRequirements(m_GamePieceSelector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_GamePieceSelector.setGamepieceYellow();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_GamePieceSelector.getCurrentGamepiece() == Gamepiece.YELLOW) {
            return true;
        } else {
            return false;
        }

    }
}
