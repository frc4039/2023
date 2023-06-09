// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.commands.PIDTranslateForAuto.OffsetNeeded;
import edu.wpi.first.wpilibj2.command.Command;

public class BumpSideThreePurple extends SequentialCommandGroup {

    public BumpSideThreePurple(RobotContainer container, boolean isRed) {
        if (isRed) {
            ExecuteComands(container, startPosition_Red, bumpOutbound_Red, bumpInbound_Red, purplePickup1_Red,
                    scoringLocation_Red, purplePickup2_Red, purplePickup1Rotate90_Red, purplePickup1Rotate270_Red);
        } else if (!isRed) {
            ExecuteComands(container, startPosition_Blue, bumpOutbound_Blue, bumpInbound_Blue, purplePickup1_Blue,
                    scoringLocation_Blue, purplePickup2_Blue, purplePickup1Rotate270_Blue, purplePickup1Rotate90_Blue);
        }
    }

    private void ExecuteComands(RobotContainer container, Pose2d startPosition, Pose2d bumpOutbound, Pose2d bumpInbound,
            Pose2d purplePickup1, Pose2d scoringLocation, Pose2d purplePickup2, Pose2d rotateTowardsPickup2,
            Pose2d rotateBack) {
        // reset pose to initial position
        addCommands(new ResetRobotPose(container.getSwerve(), startPosition));

        // flick the pre-loaded cube
        addCommands(new SequentialCommandGroup(new Command[] {
                new ConeGuideDeploy(container.getConeGuide()).withTimeout(0.25)
        }));

        // retract cone guide and pivot up to vertical, also start driving to bump
        addCommands(new ParallelRaceGroup(new Command[] {
                new ConeGuideRetract(container.getConeGuide()),
                new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                        .withTimeout(1.0)
        }));

        // extend intake and pivot to purple pickup position. Drive to purple pickup
        // location 1
        addCommands(new ParallelRaceGroup(new Command[] {
                new SeqCmdCubePickupPosition(container.getTelescopic(),
                        container.getConeGuide(),
                        container.getGripper(), container.getIntake(), container.gIntakeSpinner(),
                        container.getPivot()),
                new PIDTranslateForAuto(container.getSwerve(), purplePickup1, OffsetNeeded.None, false)
        }));

        // close gripper
        addCommands(new GripperRetrieve(container.getGripper())
                .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));
        // extend intake to avoid knocking purple out of grip
        addCommands(new IntakeExtend(
                container.getIntake(),
                container.getPivot(), false));

        // scoring position and drive back to bump
        addCommands(new ParallelCommandGroup(
                new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                        container.getPivot()),
                new SequentialCommandGroup(new Command[] {
                        new WaitUntilCommand(
                                () -> container.getPivot()
                                        .getEncoder() >= Constants.PivotConstants.kPositionForSafeIntakeRetract),
                        new IntakeRetract(container.getIntake())
                }),
                new PIDTranslateForAuto(container.getSwerve(), bumpInbound, OffsetNeeded.None, false)));

        // extend arm
        addCommands(
                new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(.4));

        // Drop score, retract, drive to second purple pickup
        addCommands(new SequentialCommandGroup(new Command[] {
                new GripperRelease(container.getGripper())
                        .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
                new ParallelCommandGroup(new Command[] {
                        new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
                        new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
                                .withTimeout(1.0),
                        new PIDTranslateForAuto(container.getSwerve(), purplePickup1, OffsetNeeded.XPlus, false)
                })
        }));

        addCommands(new PIDTranslateForAuto(container.getSwerve(), rotateTowardsPickup2, OffsetNeeded.XPlus, false));

        // extend intake and pivot to purple pickup position. Drive to purple pickup
        // location 2
        // addCommands(new ParallelRaceGroup(new Command[] {
        // new SeqCmdCubePickupPosition(container.getTelescopic(),
        // container.getConeGuide(),
        // container.getGripper(), container.getIntake(), container.gIntakeSpinner(),
        // container.getPivot()),
        // new PIDTranslateForAuto(container.getSwerve(), purplePickup2,
        // OffsetNeeded.None, false),
        // new PIDTranslateForAuto(container.getSwerve(), purplePickup1,
        // OffsetNeeded.XPlus, false),
        // new PIDTranslateForAuto(container.getSwerve(), rotateBack,
        // OffsetNeeded.XPlus, false)
        // }));

        // scoring position and drive back to bump
        // addCommands(new ParallelCommandGroup(
        // new CmdGrpScoringPosition(container.getConeGuide(),
        // container.getTelescopic(),
        // container.getPivot()),
        // new SequentialCommandGroup(new Command[] {
        // new WaitUntilCommand(
        // () -> container.getPivot()
        // .getEncoder() >= Constants.PivotConstants.kPositionForSafeIntakeRetract),
        // new IntakeRetract(container.getIntake())
        // }),
        // new PIDTranslateForAuto(container.getSwerve(), bumpInbound,
        // OffsetNeeded.None, false)));

        // extend arm
        // addCommands(
        // new TelescopicScoringExtendMid(container.getTelescopic(),
        // container.getPivot()).withTimeout(.4));

        // Drop score, retract, drive out of community
        // addCommands(new SequentialCommandGroup(new Command[] {
        // new GripperRelease(container.getGripper())
        // .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
        // new ParallelCommandGroup(new Command[] {
        // new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
        // new PivotMoveToPosition(container.getPivot(),
        // Constants.PivotConstants.kPositionTravel)
        // .withTimeout(1.0),
        // new PIDTranslateForAuto(container.getSwerve(), bumpOutbound,
        // OffsetNeeded.XPlus, false),
        // new PIDTranslateForAuto(container.getSwerve(), purplePickup1,
        // OffsetNeeded.XPlus, false)
        // })
        // }));

        // go to floor pickup for yellow to prep pickup in tele
        // addCommands(new ParallelCommandGroup(new Command[] {
        // new GripperRelease(container.getGripper())
        // .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
        // new ConeGuideDeploy(container.getConeGuide())
        // .withTimeout(Constants.ConeGuideConstants.kConeGuideRetractTimeout),
        // new PivotMoveToPosition(container.getPivot(),
        // Constants.PivotConstants.kPositionPickupCone)
        // }));
    }

    /* Red Paths */
    /* ========= */

    public static Pose2d startPosition_Red = new Pose2d(14.87 - 2.5, 1.04, Rotation2d.fromDegrees(180));
    public static Pose2d bumpOutbound_Red = new Pose2d(14.87 - 2, 1.04, Rotation2d.fromDegrees(0));
    public static Pose2d bumpInbound_Red = new Pose2d(14.87 - 2.5, 1.04, Rotation2d.fromDegrees(0));
    public static Pose2d purplePickup1_Red = new Pose2d(14.87 - 5.5, 1.04 - 0.05, Rotation2d.fromDegrees(0));
    public static Pose2d purplePickup1Rotate90_Red = new Pose2d(14.87 - 5.5, 1.04 - 0.05, Rotation2d.fromDegrees(90));
    public static Pose2d purplePickup1Rotate270_Red = new Pose2d(14.87 - 5.5, 1.04 - 0.05, Rotation2d.fromDegrees(270));
    public static Pose2d scoringLocation_Red = new Pose2d(15, 1.09, Rotation2d.fromDegrees(0)); // TODO: Same as Red
                                                                                                // Purple 1
    public static Pose2d purplePickup2_Red = new Pose2d(14.87 - 5.5, 1.04 + 1.2, Rotation2d.fromDegrees(0));

    /* Blue Paths */
    /* ========== */

    public static Pose2d startPosition_Blue = new Pose2d(1.64 + 2.5, 1.04, Rotation2d.fromDegrees(0));
    public static Pose2d bumpOutbound_Blue = new Pose2d(1.64 + 2, 1.04, Rotation2d.fromDegrees(0));
    public static Pose2d bumpInbound_Blue = new Pose2d(1.64 + 2.5, 1.04, Rotation2d.fromDegrees(0));
    public static Pose2d purplePickup1_Blue = new Pose2d(1.64 + 5.5, 1.04 - 0.05, Rotation2d.fromDegrees(0));
    public static Pose2d purplePickup1Rotate90_Blue = new Pose2d(1.64 + 5.5, 1.04 - 0.05, Rotation2d.fromDegrees(90));
    public static Pose2d purplePickup1Rotate270_Blue = new Pose2d(1.64 + 5.5, 1.04 - 0.05, Rotation2d.fromDegrees(270));
    public static Pose2d scoringLocation_Blue = new Pose2d(1.58, 1.03, Rotation2d.fromDegrees(0)); // TODO: Same as Blue
                                                                                                   // Purple 3
                                                                                                   // Possibly need to
                                                                                                   // decrease 'x' value
    public static Pose2d purplePickup2_Blue = new Pose2d(1.64 + 5.5, 1.04 + 1.2, Rotation2d.fromDegrees(0));
}