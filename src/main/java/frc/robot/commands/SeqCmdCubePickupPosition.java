// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConeGuideConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ConeGuide;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeSpinner;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

//Gets the robot ready to pick up a Cube deploying the Intake
public class SeqCmdCubePickupPosition extends ParallelCommandGroup {
    public SeqCmdCubePickupPosition(Telescopic s_Telescopic, ConeGuide s_ConeGuide, Gripper s_Gripper, Intake s_Intake,
            IntakeSpinner s_IntakeSpinner,
            Pivot s_Pivot) {
        addCommands(
                new TelescopicRetract(s_Telescopic),
                new ConeGuideRetract(s_ConeGuide).withTimeout(ConeGuideConstants.kConeGuideRetractTimeout),
                new GripperRelease(s_Gripper).withTimeout(GripperConstants.kGripperReleaseTimeout),
                new IntakeMotorSpin(s_IntakeSpinner),
                new IntakePickup(s_Intake),
                new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionPickupCubeWithIntake)
        // new ConditionalCommand(new PivotMoveToPosition(s_Pivot,
        // PivotConstants.kPositionPrePickupCube),
        // new PivotMoveToPosition(s_Pivot,
        // PivotConstants.kPositionPickupCubeWithIntake),
        // () -> !s_Pivot.atSetpoint(PivotConstants.kPositionPrePickupCube)
        // || !s_Intake.atSetpoint(IntakeConstants.kIntakePositionRightPickup))
        );
    }
}