// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConeGuideConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ConeGuide;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeqCmdCubePickupPosition extends SequentialCommandGroup {
    /** Creates a new CubePickup. */
    public SeqCmdCubePickupPosition(Telescopic s_Telescopic, ConeGuide s_ConeGuide, Gripper s_Gripper, Intake s_Intake,
            Pivot s_Pivot) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new TelescopicRetract(s_Telescopic),
                new ConeGuideRetract(s_ConeGuide).withTimeout(ConeGuideConstants.kConeGuideRetractTimeout),
                new GripperRelease(s_Gripper).withTimeout(GripperConstants.kGripperReleaseTimeout),
                new IntakePickup(s_Intake),
                new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionPickupCube));
    }
}
