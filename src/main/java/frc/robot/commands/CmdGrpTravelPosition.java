// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ConeGuide;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

//Gets the robot subsystems in travel mode- that is, everything retracted and pivot to vertical position*/
public class CmdGrpTravelPosition extends SequentialCommandGroup {

    public CmdGrpTravelPosition(Telescopic s_Telescopic, ConeGuide s_ConeGuide, Pivot s_Pivot, Intake s_Intake) {
        addCommands(
                new IntakeExtend(s_Intake, s_Pivot, false).withTimeout(2),
                new ParallelCommandGroup(new Command[] {
                        new TelescopicRetract(s_Telescopic),
                        new ConeGuideRetract(s_ConeGuide)
                                .withTimeout(Constants.ConeGuideConstants.kConeGuideRetractTimeout),
                        new PivotMoveToPosition(s_Pivot, PivotConstants.kPositionTravel),
                        new SequentialCommandGroup(new Command[] {
                                new WaitUntilCommand(
                                        () -> s_Pivot.getEncoder() >= PivotConstants.kPositionForSafeIntakeRetract),
                                new IntakeRetract(s_Intake)
                        })
                }));
    }
}
