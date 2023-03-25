// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.BlinkinGamePiece;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreYellowHigh extends SequentialCommandGroup {
    /** Creates a new ScoreYellowHigh. */
    public ScoreYellowHigh(Pivot s_Pivot, Gripper s_Gripper, Telescopic s_Telescopic,
            BlinkinGamePiece s_BlinkinGamePiece) {
        addCommands(
                // pivot to extend angle
                new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionScoringCone),
                // extend to far
                new TelescopicScoringExtendFar(s_Telescopic, s_Pivot),
                // score
                new CmdGrpGamePieceScoring(s_Pivot, s_Gripper, s_Telescopic, s_BlinkinGamePiece));
    }
}
