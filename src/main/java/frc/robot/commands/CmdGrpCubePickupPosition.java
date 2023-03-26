// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ConeGuide;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

//Gets the robot ready to pick up a Cube
public class CmdGrpCubePickupPosition extends ParallelCommandGroup {

    public CmdGrpCubePickupPosition(Telescopic s_Telescopic, ConeGuide s_ConeGuide, Gripper s_Gripper, Intake s_Intake,
            Pivot s_Pivot) {
        addCommands(
                new TelescopicRetract(s_Telescopic),
                new ParallelCommandGroup(new Command[] {
                        new ConeGuideRetract(s_ConeGuide),
                        new GripperRelease(s_Gripper),
                        new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionPickupCube)
                }));
    }
}
