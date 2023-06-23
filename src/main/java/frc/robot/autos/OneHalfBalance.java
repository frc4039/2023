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

public class OneHalfBalance extends SequentialCommandGroup {

    public OneHalfBalance(RobotContainer container) {
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
                        .withTimeout(1.0) }));

        // start driving over charge station. After 1.5 seconds move pivot to pickup and
        // extend intake. Command should end when end of path reached.
        addCommands(new ParallelRaceGroup(new Command[] {
                new SequentialCommandGroup(new Command[] {
                        new WaitCommand(1.5),
                        new SeqCmdCubePickupPosition(container.getTelescopic(),
                                container.getConeGuide(),
                                container.getGripper(), container.getIntake(), container.gIntakeSpinner(),
                                container.getPivot())
                }),
                AutoFollowPath.createFollowCommand(container.getSwerve(),
                        pDropToMobility)
        }));

        // close gripper
        addCommands(new GripperRetrieve(container.getGripper())
                .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout));
        // extend intake to avoid interference
        addCommands(new IntakeExtend(
                container.getIntake(),
                container.getPivot(), false));

        // pivot to scoring position, then move intake back in, and drive back to charge
        // station
        addCommands(new ParallelCommandGroup(
                new CmdGrpScoringPosition(container.getConeGuide(),
                        container.getPivot()),
                new SequentialCommandGroup(new Command[] {
                        new WaitUntilCommand(
                                () -> container.getPivot()
                                        .getEncoder() >= Constants.PivotConstants.kPositionForSafeIntakeRetract),
                        new IntakeRetract(container.getIntake())
                }),
                AutoFollowPath.createFollowCommand(container.getSwerve(),
                        pPickupToCharge)));
        // drive up charge station and balance
        addCommands(new ParallelRaceGroup(AutoFollowPath.createFollowCommand(container.getSwerve(),
                pDropAndBalanceYellowSide1),
                new AwaitLevelCharge(container.getSwerve())));
    }

    public static Trajectory pDropToMobility = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(5.5, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.chargeStationForwardConfigHalfGamepiece);

    public static Trajectory pPickupToCharge = TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.5, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4.25, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.chargeStationReverseConfigHhalfGamepiece);

    public static Trajectory pDropAndBalanceYellowSide1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.25, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(2.0, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.balanceReverseConfig);
}
