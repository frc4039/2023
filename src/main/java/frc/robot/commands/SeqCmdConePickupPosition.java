// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescopic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeqCmdConePickupPosition extends SequentialCommandGroup {
  /** Creates a new ConePickup. */
  public SeqCmdConePickupPosition(Telescopic s_Telescopic, Gripper s_Gripper, Pivot s_Pivot, Intake s_Intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescopicRetract(s_Telescopic),
      new GripperRelease(s_Gripper).withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
      new PivotMoveToPosition(s_Pivot, Constants.PivotConstants.positionPickupCone),
      new IntakeMotorStop(s_Intake),
      new IntakeRetract(s_Intake)
    );
  }
}
