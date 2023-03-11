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
public class DropAndDriveAndPickup extends SequentialCommandGroup {

    public DropAndDriveAndPickup(RobotContainer container) {
        addCommands(
                (new CmdGrpTravelPosition(container.getTelescopic(),
                        container.getConeGuide(), container.getPivot(),
                        container.getIntake())).withTimeout(1));
        addCommands(
                (new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                        container.getPivot())).withTimeout(4));
        addCommands((new GripperRelease(container.getGripper())).withTimeout(2));
        addCommands(
                (new CmdGrpTravelPosition(container.getTelescopic(),
                        container.getConeGuide(), container.getPivot(),
                        container.getIntake())).withTimeout(3));
        addCommands(new ResetRobotPose(container.getSwerve(), path.getInitialPose()));
        addCommands(AutoFollowPath.createFollowCommand(container.getSwerve(), path));
        addCommands(new CmdGrpConePickupPosition(container.getTelescopic(), container.getGripper(), container.getConeGuide(), container.getPivot(), container.getIntake()));
        addCommands(new GripperRetrieve(container.getGripper()));
        addCommands(new ParallelCommandGroup(new CmdGrpTravelPosition(container.getTelescopic(), container.getConeGuide(), container.getPivot(), container.getIntake()), 
                    AutoFollowPath.createFollowCommand(container.getSwerve(), step2)));
                    addCommands(
                (new CmdGrpScoringPosition(container.getConeGuide(), container.getTelescopic(),
                        container.getPivot())).withTimeout(4));
        addCommands((new GripperRelease(container.getGripper())).withTimeout(2));
        addCommands(
                (new CmdGrpTravelPosition(container.getTelescopic(),
                        container.getConeGuide(), container.getPivot(),
                        container.getIntake())).withTimeout(3));
    }

    public static Trajectory path = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1.5, 0)),
            new Pose2d(4.5, 0, Rotation2d.fromDegrees(180)),
            Constants.AutoConstants.forwardConfig);

    public static Trajectory step2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1.5, 0)),
                new Pose2d(4.5, 0, Rotation2d.fromDegrees(180)),
                Constants.AutoConstants.reverseConfig);
}
