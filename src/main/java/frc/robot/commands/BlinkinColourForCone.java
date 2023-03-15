// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.subsystems.BlinkinGamePiece;

public class BlinkinColourForCone extends CommandBase {
    private BlinkinGamePiece m_BlinkinGamePiece;

    /** Creates a new BlinkinColourForCone. */
    public BlinkinColourForCone(BlinkinGamePiece blinkinGamePiece) {
        m_BlinkinGamePiece = blinkinGamePiece;
        addRequirements(m_BlinkinGamePiece);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_BlinkinGamePiece.SetColour(BlinkinConstants.kColourValueCone);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
