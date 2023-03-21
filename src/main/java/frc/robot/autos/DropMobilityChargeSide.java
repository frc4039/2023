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

public class DropMobilityChargeSide extends SequentialCommandGroup {

    public DropMobilityChargeSide(RobotContainer container) {
        // reset initial pose TODO: does this need to be changed now that we use April
        // tags?
        addCommands(new ResetRobotPose(container.getSwerve(), pDropToMobility.getInitialPose()));

        // Pivot and extend arm
        addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
        addCommands(new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));

        // drop pivot and open gripper
        addCommands(
                new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease));
        addCommands(new GripperRelease(container.getGripper())
                .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

        // retract arm and pivot up to vertical, also start driving
        addCommands(new ParallelCommandGroup(new Command[] {
                new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                        .withTimeout(1.0),
                AutoFollowPath.createFollowCommand(container.getSwerve(),
                        pDropToMobility) }));
    }

    public static Trajectory pDropToMobility = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(5.0, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);
}
