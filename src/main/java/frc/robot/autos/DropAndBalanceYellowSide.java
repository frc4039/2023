// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

public class DropAndBalanceYellowSide extends SequentialCommandGroup {

    public DropAndBalanceYellowSide(RobotContainer container) {
        addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
        addCommands(new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
        addCommands(new SequentialCommandGroup(new Command[] {
                new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
                new GripperRelease(container.getGripper())
                        .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                new ParallelCommandGroup(new Command[] {
                        new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                        new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                .withTimeout(1.0) }) }));
        addCommands(new ResetRobotPose(container.getSwerve(), pDropAndBalanceYellowSide1.getInitialPose()));
        addCommands(new ParallelRaceGroup(AutoFollowPath.createFollowCommand(container.getSwerve(),
                pDropAndBalanceYellowSide1),
                new AwaitLevelCharge(container.getSwerve())));
    }

    public static Trajectory pDropAndBalanceYellowSide1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4.0, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.balanceForwardConfig);
}
