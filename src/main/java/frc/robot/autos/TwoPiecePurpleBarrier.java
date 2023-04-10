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
import frc.robot.commands.PIDTranslateForAuto.OffsetNeeded;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TwoPiecePurpleBarrier extends SequentialCommandGroup {

    public TwoPiecePurpleBarrier(RobotContainer container, boolean isRed) {
        if (isRed) {
            // addCommands(new ResetRobotPose(container.getSwerve(),
            // middlePath_1_Red.getInitialPose()));

            // reset pose to initial position
            addCommands(new ResetRobotPose(container.getSwerve(), startPosition_Red));

            // Pivot and extend arm -> score preload
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCube));
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(.55));
            // Open gripper
            addCommands(new GripperRelease(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // retract arm and pivot up to vertical, also start driving to middle to pickup
            // purple
            addCommands(new ParallelCommandGroup(new Command[] {
                    new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionGreenCubePickup),
                    new PIDTranslateForAuto(container.getSwerve(), purplePickup_Red, OffsetNeeded.None, false)

            }));

            // close gripper
            addCommands(new GripperRetrieve(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // scoring position and drive back to score
            addCommands(new ParallelCommandGroup(
                    new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                            container.getPivot()),
                    // AutoFollowPath.createFollowCommandNoStop(container.getSwerve(),
                    // returnPath_1_Red_pt1)
                    new PIDTranslateForAuto(container.getSwerve(), scoringLocation_Red, OffsetNeeded.Y, true)));
            // addCommands(new PivotMoveToPosition(container.getPivot(),
            // Constants.PivotConstants.kPositionScoringCone));

            // addCommands(new
            // InstantCommand(container.getSwerve()::enableAutoVisionTracking));
            // addCommands(new PIDTranslateForAuto(container.getSwerve(),
            // scoringLocation_Red, false));
            // addCommands(new
            // InstantCommand(container.getSwerve()::disableAutoVisionTracking));

            // extend arm
            addCommands(
                    new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(0.4));
            // Drop score, retract, drive to yellow pickup
            addCommands(new SequentialCommandGroup(new Command[] {
                    // new PivotMoveToPosition(container.getPivot(),
                    // Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                            new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                            new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                    .withTimeout(1.0),
                            // AutoFollowPath.createFollowCommand(container.getSwerve(), middlePath_2_Red)
                            new PIDTranslateForAuto(container.getSwerve(), yellowPickup_Red, OffsetNeeded.X, false)
                    }) }));

            // go to floor pickup for yellow to prep pickup in tele
            addCommands(new ParallelCommandGroup(new Command[] {
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ConeGuideDeploy(container.getConeGuide())
                            .withTimeout(Constants.ConeGuideConstants.kConeGuideRetractTimeout),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionPickupCone)
            }));
            ;
        } else if (!isRed) {
            // addCommands(new ResetRobotPose(container.getSwerve(),
            // middlePath_1_Blue.getInitialPose()));
            addCommands(new ResetRobotPose(container.getSwerve(), startPosition_Blue));

            // Pivot and extend arm
            addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCube));
            addCommands(
                    new TelescopicScoringExtendFar(container.getTelescopic(), container.getPivot()).withTimeout(.55));
            // Open gripper
            addCommands(new GripperRelease(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // addCommands(new
            // InstantCommand(container.getSwerve()::enableAutoVisionTracking));

            // retract arm and pivot up to vertical, also start driving to middle
            addCommands(new ParallelCommandGroup(new Command[] {
                    new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionGreenCubePickup),
                    // new TelescopicGreenCube(container.getTelescopic()).withTimeout(1.0),
                    // AutoFollowPath.createFollowCommand(container.getSwerve(),
                    // middlePath_1_Blue)
                    new PIDTranslateForAuto(container.getSwerve(), purplePickup_Blue, OffsetNeeded.None, false)
            }));

            // close gripper
            addCommands(new GripperRetrieve(container.getGripper())
                    .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));

            // pivot to scoring position and drive back to scoring location
            addCommands(new ParallelCommandGroup(
                    new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                            container.getPivot()),
                    // AutoFollowPath.createFollowCommandNoStop(container.getSwerve(),
                    // returnPath_1_Blue_pt1)
                    new PIDTranslateForAuto(container.getSwerve(), scoringLocation_Blue, OffsetNeeded.Y, true)));
            // addCommands(new PivotMoveToPosition(container.getPivot(),
            // Constants.PivotConstants.kPositionScoringCone));

            // addCommands(new
            // InstantCommand(container.getSwerve()::enableAutoVisionTracking));
            // addCommands(new PIDTranslateForAuto(container.getSwerve(),
            // scoringLocation_Blue, false));
            // addCommands(new
            // InstantCommand(container.getSwerve()::disableAutoVisionTracking));

            // extend arm
            addCommands(
                    new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(1.0));

            // Score second game piece and druve to yellow
            addCommands(new SequentialCommandGroup(new Command[] {
                    // new PivotMoveToPosition(container.getPivot(),
                    // Constants.PivotConstants.kPositionScoringRelease),
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ParallelCommandGroup(new Command[] {
                            new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                            new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                    .withTimeout(1.0),
                            // AutoFollowPath.createFollowCommand(container.getSwerve(), middlePath_2_Blue)
                            new PIDTranslateForAuto(container.getSwerve(), yellowPickup_Blue, OffsetNeeded.X, false)
                    }) }));

            // get ready for yellow floor pickup
            addCommands(new ParallelCommandGroup(new Command[] {
                    new GripperRelease(container.getGripper())
                            .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                    new ConeGuideDeploy(container.getConeGuide())
                            .withTimeout(Constants.ConeGuideConstants.kConeGuideRetractTimeout),
                    new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionPickupCone)
            }));
            ;
        }
    }

    /* Red Paths */ // 0.0572
    /* ========= */

    public static Pose2d startPosition_Red = new Pose2d(14.87, 4.38, Rotation2d.fromDegrees(180));
    public static Pose2d purplePickup_Red = new Pose2d(14.87 - 5.1, 4.38 + 0.05, Rotation2d.fromDegrees(0));
    // public static Pose2d returnPath_1_Red_pt1_pose = new Pose2d(14.87 - 1.5, 4.38
    // - 0, Rotation2d.fromDegrees(0));
    public static Pose2d scoringLocation_Red = new Pose2d(14.87, 4.44, Rotation2d.fromDegrees(0)); // this should be the
                                                                                                   // same as red purple
                                                                                                   // 3
    // public static Pose2d middlePath_2_Red_pose_pt1 = new Pose2d(14.87 - 5.8, 4.38
    // - 0, Rotation2d.fromDegrees(0));
    public static Pose2d yellowPickup_Red = new Pose2d(14.87 - 6, 4.38 - 1.4, Rotation2d.fromDegrees(0));

    /*
     * =============================================================================
     */
    /* Blue Paths */
    /* ========== */

    public static Pose2d startPosition_Blue = new Pose2d(1.64 + 0, 4.35 + 0, Rotation2d.fromDegrees(0));
    public static Pose2d purplePickup_Blue = new Pose2d(1.64 + 5.1, 4.35 + 0.1, Rotation2d.fromDegrees(0));
    // public static Pose2d returnPath_1_Blue_pt1_pose = new Pose2d(1.64 + 1.5, 4.35
    // + 0.05, Rotation2d.fromDegrees(0));
    public static Pose2d scoringLocation_Blue = new Pose2d(1.64, 4.4, Rotation2d.fromDegrees(0)); // this should be the
                                                                                                  // same as purple 1
    // public static Pose2d middlePath_2_Blue_pose_pt1 = new Pose2d(1.64 + 6.3, 4.35
    // + 0, Rotation2d.fromDegrees(0));
    public static Pose2d yellowPickup_Blue = new Pose2d(1.64 + 6, 4.35 + -1.4, Rotation2d.fromDegrees(0));
}