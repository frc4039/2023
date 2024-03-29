// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BlinkinGamePiece;
import frc.robot.subsystems.GamePieceSelector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CmdGrpSetYellow extends SequentialCommandGroup {
    /** Creates a new CmdGrpSetYellow. */
    public CmdGrpSetYellow(GamePieceSelector s_GamePieceSelector,
            BlinkinGamePiece s_BlinkinGamePiece) {
        addCommands(
                new SetRobotStateYellow(s_GamePieceSelector),
                new BlinkinColourForCone(s_BlinkinGamePiece));

    }
}
