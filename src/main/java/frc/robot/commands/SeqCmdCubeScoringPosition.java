// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeqCmdCubeScoringPosition extends SequentialCommandGroup {
  /** Creates a new ConeScoringPosition. */
  public SeqCmdCubeScoringPosition(Telescopic s_Telescopic, Pivot s_Pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.positionScoringCube),
        new TelescopicExtendFar(s_Telescopic));
  }
}