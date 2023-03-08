// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ConeGuideConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ConeGuide;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

public class SeqCmdConePickupPosition extends SequentialCommandGroup {

    public SeqCmdConePickupPosition(Telescopic s_Telescopic, Gripper s_Gripper, ConeGuide s_ConeGuide, Pivot s_Pivot,
            Intake s_Intake) {
        addCommands(
                new TelescopicRetract(s_Telescopic),
                new ParallelCommandGroup(new Command[] {
                        new GripperRelease(s_Gripper)
                                .withTimeout(GripperConstants.kGripperReleaseTimeout),
                        new ConeGuideDeploy(s_ConeGuide)
                                .withTimeout(ConeGuideConstants.kConeGuideRetractTimeout),
                        new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionPickupCone)
                }));
    }
}
