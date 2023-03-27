// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ConeGuide;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

//Gets the robot ready to pick up a Cone
public class CmdGrpConePickupPosition extends SequentialCommandGroup {

    public CmdGrpConePickupPosition(Telescopic s_Telescopic, Gripper s_Gripper, ConeGuide s_ConeGuide, Pivot s_Pivot,
            Intake s_Intake) {
        addCommands(
                new TelescopicRetract(s_Telescopic),
                new ParallelCommandGroup(new Command[] {
                        new GripperRelease(s_Gripper),
                        new ConeGuideDeploy(s_ConeGuide)
                                .withTimeout(Constants.ConeGuideConstants.kConeGuideDeployTimeout),
                        new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionPickupCone)
                }));
    }
}
