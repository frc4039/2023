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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropAndDriveAndPickup extends SequentialCommandGroup {

    public DropAndDriveAndPickup(RobotContainer container) {
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
        addCommands(new ResetRobotPose(container.getSwerve(), backoutPath_step1.getInitialPose()));
        addCommands(AutoFollowPath.createFollowCommand(container.getSwerve(), backoutPath_step1));
        addCommands(new TeleopSwerveAtFixedRotation(container.getSwerve(), () -> 0, () -> 0, 180).withTimeout(1.5));
        addCommands(AutoFollowPath.createFollowCommand(container.getSwerve(), backoutPath_step2));
        addCommands(new CmdGrpConePickupPosition(container.getTelescopic(), container.getGripper(), container.getConeGuide(), container.getPivot(), container.getIntake()));
        addCommands(new WaitCommand(0.5));
        addCommands(new GripperRetrieve(container.getGripper())
                .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));
        addCommands(new WaitCommand(0.1));
        addCommands(new CmdGrpTravelPosition(container.getTelescopic(), container.getConeGuide(), container.getPivot(), container.getIntake()));
        addCommands(new TeleopSwerveAtFixedRotation(container.getSwerve(), () -> 0, () -> 0, 0).withTimeout(1.5));
        addCommands(AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath));
        addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
        addCommands(new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
        addCommands(new SequentialCommandGroup(new Command[] {
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                    new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                            .withTimeout(1.0) }) }));;
        
    }

    public static Trajectory backoutPath_step1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory backoutPath_step2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(4.5, 0.1, Rotation2d.fromDegrees(0)),
                Constants.AutoConstants.forwardConfig);

    public static Trajectory returnPath = TrajectoryGenerator.generateTrajectory(
                new Pose2d(4.5, 0.1, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                Constants.AutoConstants.reverseConfig);
}
