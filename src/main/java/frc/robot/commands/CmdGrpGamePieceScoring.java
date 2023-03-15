// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.BlinkinGamePiece;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

//Makes the robot release a game piece
public class CmdGrpGamePieceScoring extends SequentialCommandGroup {

    public CmdGrpGamePieceScoring(Pivot s_Pivot, Gripper s_Gripper, Telescopic s_Telescopic,
            BlinkinGamePiece s_BlinkinGamePiece) {
        addCommands(
                new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionScoringRelease),
                new GripperRelease(s_Gripper).withTimeout(GripperConstants.kGripperReleaseTimeout),
                new ParallelCommandGroup(new Command[] {
                        new TelescopicRetract(s_Telescopic),
                        new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionTravel)
                }), new BlinkinRainbow(s_BlinkinGamePiece));
    }
}
