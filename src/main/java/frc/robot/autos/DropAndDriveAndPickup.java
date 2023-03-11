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
import frc.robot.commands.AutoFollowPath;
import frc.robot.commands.GripperRelease;
import frc.robot.commands.GripperRetrieve;
import frc.robot.commands.ResetRobotPose;
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
        addCommands(new ResetRobotPose(container.getSwerve(), path.getInitialPose()));
        addCommands(AutoFollowPath.createFollowCommand(container.getSwerve(), path));
        addCommands(new CmdGrpConePickupPosition(container.getTelescopic(), container.getGripper(), container.getConeGuide(), container.getPivot(), container.getIntake()));
        addCommands(new WaitCommand(0.5));
        addCommands(new GripperRetrieve(container.getGripper()));
        addCommands(new CmdGrpTravelPosition(container.getTelescopic(), container.getConeGuide(), container.getPivot(), container.getIntake())); 
        addCommands(AutoFollowPath.createFollowCommand(container.getSwerve(), step2));
        // addCommands(new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringCone));
        // addCommands(new TelescopicScoringExtendMid(container.getTelescopic(), container.getPivot()).withTimeout(1.0));
        // addCommands(new SequentialCommandGroup(new Command[] {
        //             new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionScoringRelease),
        //             new GripperRelease(container.getGripper())
        //                     .withTimeout(Constants.GripperConstants.kGripperReleaseTimeout),
        //             new ParallelCommandGroup(new Command[] {
        //             new TelescopicRetract(container.getTelescopic()).withTimeout(1.0),
        //             new PivotMoveToPosition(container.getPivot(), Constants.PivotConstants.kPositionTravel)
        //                     .withTimeout(1.0) }) }));;
        
    }

    public static Trajectory path = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(),
            new Pose2d(4.5, 0, Rotation2d.fromDegrees(0)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory step2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(4.5, 0, Rotation2d.fromDegrees(0)),
                Constants.AutoConstants.reverseConfig);
}
