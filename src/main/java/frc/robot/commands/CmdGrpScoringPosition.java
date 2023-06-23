// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.ConeGuide;

//Gets the robot ready for scoring a game piece
public class CmdGrpScoringPosition extends SequentialCommandGroup {

    public CmdGrpScoringPosition(ConeGuide s_ConeGuide, Pivot s_Pivot) {
        addCommands(
                new ConeGuideRetract(s_ConeGuide).withTimeout(Constants.ConeGuideConstants.kConeGuideRetractTimeout),
                new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionScoringCone));
    }
}
