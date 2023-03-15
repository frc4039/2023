// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BlinkinGamePiece;
import frc.robot.subsystems.Gripper;

//Releases the gripper and set the blinkin to Rainbow pattern
public class CmdGrpGripperRelease extends ParallelCommandGroup {
    public CmdGrpGripperRelease(Gripper s_Gripper, BlinkinGamePiece s_BlinkinGamePiece) {
        addCommands(
                new GripperRelease(s_Gripper),
                new BlinkinRainbow(s_BlinkinGamePiece));
    }
}
