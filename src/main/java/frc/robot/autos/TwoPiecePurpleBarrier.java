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

public class TwoPiecePurpleBarrier extends SequentialCommandGroup {

    public TwoPiecePurpleBarrier(RobotContainer container, boolean isRed) {
        if (isRed) {
            addCommands(new ResetRobotPose(container.getSwerve(), middlePath_1_Red.getInitialPose()));

            // Pivot and extend arm
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCube));
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(.55));
            // Open gripper
            addCommands(new GripperRelease(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // retract arm and pivot up to vertical, also start driving to middle
            addCommands(new ParallelCommandGroup(new Command[] {
                    new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionGreenCubePickup),
                    // new TelescopicGreenCube(container.getTelescopic()).withTimeout(1.0),
                    AutoFollowPath.createFollowCommand(container.getSwerve(),
                            middlePath_1_Red) }));

            // close gripper
            addCommands(new GripperRetrieve(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // scoring position and drive back to score
            addCommands(new ParallelCommandGroup(
                    new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                            container.getPivot()),
                    AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_1_Red)));
            // addCommands(new PivotMoveToPosition(container.getPivot(),
            // Constants.PivotConstants.kPositionScoringCone));

            // extend arm
            addCommands(
                    new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(0.4));
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
                            AutoFollowPath.createFollowCommand(container.getSwerve(), middlePath_2_Red) }) }));

            addCommands(new ParallelCommandGroup(new Command[] {
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ConeGuideDeploy(container.getConeGuide())
                            .withTimeout(Constants.ConeGuideConstants.kConeGuideRetractTimeout),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionPickupCone)
            }));
            ;
        } else if (!isRed) {
            addCommands(new ResetRobotPose(container.getSwerve(), middlePath_1_Blue.getInitialPose()));

            // Pivot and extend arm
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1));

            // drop pivot and open gripper
            addCommands(
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease));
            addCommands(new GripperRelease(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // retract arm and pivot up to vertical, also start driving to middle
            addCommands(new ParallelCommandGroup(new Command[] {
                    new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionGreenCubePickup),
                    // new TelescopicGreenCube(container.getTelescopic()).withTimeout(1.0),
                    AutoFollowPath.createFollowCommand(container.getSwerve(),
                            middlePath_1_Blue) }));

            // close gripper
            addCommands(new GripperRetrieve(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // scoring position and drive back to score
            addCommands(new ParallelCommandGroup(
                    new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                            container.getPivot()),
                    AutoFollowPath.createFollowCommand(container.getSwerve(), returnPath_1_Blue)));
            // addCommands(new PivotMoveToPosition(container.getPivot(),
            // Constants.PivotConstants.kPositionScoringCone));

            // extend arm
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
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
                            AutoFollowPath.createFollowCommand(container.getSwerve(), middlePath_2_Blue) }) }));
        }
    }

    /* Red Paths */
    /* ========= */
    public static Trajectory middlePath_1_Red = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4.65, -0.2, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.twoPurpleBarrierForwardConfig);

    public static Trajectory returnPath_1_Red = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.65, -0.2, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.twoPurpleBarrierReverseConfig);

    public static Trajectory middlePath_2_Red = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(5.5, 0)),
            new Pose2d(5.5, 1.2, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.twoPurpleBarrierForwardConfigFast);
    /*
     * =============================================================================
     */
    /* Blue Paths */
    /* ========== */
    public static Trajectory middlePath_1_Blue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(2, 0)),
            new Pose2d(4.75, -0.381, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory returnPath_1_Blue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.75, -0.381, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1.2, 0)),
            new Pose2d(0.3, -0.6, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.reverseConfig);

    public static Trajectory middlePath_2_Blue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 0)),
            new Pose2d(5, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

}