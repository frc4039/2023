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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropAndDriveAndStrafeAndPickup extends SequentialCommandGroup {

    public DropAndDriveAndStrafeAndPickup(RobotContainer container) {

        if (DriverStation.getAlliance().toString() == "Red") {
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
            addCommands(new SequentialCommandGroup(new Command[] {
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                            new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                            new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                    .withTimeout(1.0) }) }));
            addCommands(new ResetRobotPose(container.getSwerve(), strafePath_step1_Red.getInitialPose()));
            addCommands(new ParallelCommandGroup(
                    AutoFollowPath.createFollowCommand(container.getSwerve(), strafePath_step1_Red),
                    new CmdGrpConePickupPosition(container.getTelescopic(), container.getGripper(),
                            container.getConeGuide(), container.getPivot(), container.getIntake())));
            addCommands(
                    new ParallelCommandGroup(
                            AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_1_Red)),
                    new GripperRetrieve(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));
            addCommands(new ParallelCommandGroup(
                    new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                            container.getPivot()),
                    AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_2_Red)));
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
            addCommands(
                    new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
            addCommands(new SequentialCommandGroup(new Command[] {
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                            new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                            new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                    .withTimeout(1.0) }) }));
            ;
        } else if (DriverStation.getAlliance().toString() == "Blue") {
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
            addCommands(new SequentialCommandGroup(new Command[] {
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                            new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                            new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                    .withTimeout(1.0) }) }));
            addCommands(new ResetRobotPose(container.getSwerve(), strafePath_step1_Blue.getInitialPose()));
            addCommands(new ParallelCommandGroup(
                    AutoFollowPath.createFollowCommand(container.getSwerve(), strafePath_step1_Blue),
                    new CmdGrpConePickupPosition(container.getTelescopic(), container.getGripper(),
                            container.getConeGuide(), container.getPivot(), container.getIntake())));
            addCommands(
                    new ParallelCommandGroup(
                            AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_1_Blue)),
                    new GripperRetrieve(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));
            addCommands(new ParallelCommandGroup(
                    new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                            container.getPivot()),
                    AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_2_Blue)));
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
            addCommands(
                    new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
            addCommands(new SequentialCommandGroup(new Command[] {
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                            new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                            new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                    .withTimeout(1.0) }) }));
            ;
        }
    }

    public static Trajectory strafePath_step1_Red = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(3.5, 0), new Translation2d(5, -0.7), new Translation2d(6, -0.7)),
            new Pose2d(6, 0.3, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory returnPath_1_Red = TrajectoryGenerator.generateTrajectory(
            new Pose2d(6, 0.3, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4.5, 0.3, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.reverseConfig);

    public static Trajectory returnPath_2_Red = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.5, 0.3, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(0, 0.1, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.reverseConfig);

    public static Trajectory strafePath_step1_Blue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(3.5, 0), new Translation2d(5, -0.7), new Translation2d(6, -0.7)),
            new Pose2d(6, 0.3, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory returnPath_1_Blue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(6, 0.3, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4.5, 0.3, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.reverseConfig);

    public static Trajectory returnPath_2_Blue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.5, 0.3, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(0, 0.1, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.reverseConfig);
}