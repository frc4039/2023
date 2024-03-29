// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;

public class MiddlePickupBalance extends SequentialCommandGroup {

    public MiddlePickupBalance(RobotContainer container) {
        addCommands(new ResetRobotPose(container.getSwerve(), middlePath_1.getInitialPose()));

        // Pivot and extend arm
        addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
        addCommands(new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));

        // drop pivot and open gripper
        addCommands(
                new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease));
        addCommands(new GripperRelease(container.getGripper())
                .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

        // retract arm and pivot up to vertical, also start driving to middle
        addCommands(new ParallelCommandGroup(new Command[] {
                new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionGreenCubePickup)
                        .withTimeout(1.0),
                AutoFollowPath.createFollowCommand(container.getSwerve(),
                        middlePath_1) }));

        // close gripper
        addCommands(new GripperRetrieve(container.getGripper())
                .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

        // scoring position and drive back to score
        addCommands(new ParallelCommandGroup(
                new CmdGrpScoringPosition(container.getConeGuide(), container.getPivot()),
                AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_1)));
        // addCommands(new PivotMoveToPosition(container.getPivot(),
        // Constants.PivotConstants.kPositionScoringCone));

        // extend arm
        addCommands(new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
        // Drop score, and retract
        addCommands(new SequentialCommandGroup(new Command[] {
                // new PivotMoveToPosition(container.getPivot(),
                // Constants.PivotConstants.kPositionScoringRelease),
                new GripperRelease(container.getGripper())
                        .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                new ParallelCommandGroup(new Command[] {
                        new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                        new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                .withTimeout(1.0),
                        AutoFollowPath.createFollowCommand(container.getSwerve(), middlePath_2) }) }));
        addCommands(new ParallelRaceGroup(AutoFollowPath.createFollowCommand(container.getSwerve(),
                balancePath_1),
                new AwaitLevelCharge(container.getSwerve())));

    }

    public static Trajectory middlePath_1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(5, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory returnPath_1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 0)),
            new Pose2d(0, 0.5, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.reverseConfig);

    public static Trajectory middlePath_2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0.5, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 2)),
            new Pose2d(2.2, 2, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);
    public static Trajectory balancePath_1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2.2, 2, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 2)),
            new Pose2d(3, 2, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.balanceForwardConfig);

}