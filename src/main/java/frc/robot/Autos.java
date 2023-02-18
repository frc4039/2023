package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;

public class Autos {
    private RobotContainer container;

    private TrajectoryConfig forwardConfig = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
    private TrajectoryConfig reverseConfig = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);

    private ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    public Autos(RobotContainer container) {
        this.container = container;
    }

    public SequentialCommandGroup dropNDrive() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                (new SeqCmdTravelPosition(container.getTelescopic(),
                        container.getConeGuide(), container.getPivot(),
                        container.getIntake())).withTimeout(1));
        command.addCommands(
                (new SeqCmdCubeScoringPosition(container.getTelescopic(),
                        container.getPivot()).withTimeout(4)));
        command.addCommands((new GripperRelease(container.getGripper())).withTimeout(2));
        command.addCommands(
                (new SeqCmdTravelPosition(container.getTelescopic(),
                        container.getConeGuide(), container.getPivot(),
                        container.getIntake())).withTimeout(3));
        command.addCommands(follow(container.getSwerve(), tDropNDrive[0]));

        reset(container.getSwerve(), tDropNDrive[0]);
        return command;
    }

    private Command follow(Swerve swerve, Trajectory trajectory) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);
        return swerveControllerCommand;
    }

    private Trajectory[] tDropNDrive = new Trajectory[] {
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(3.75, 0, new Rotation2d(0)),
                    List.of(
                            new Translation2d(1.5, 0)),
                    new Pose2d(0, 0, new Rotation2d(0)),
                    reverseConfig),
    };

    private void reset(Swerve swerve, Trajectory trajectory) {
        swerve.setPoseEstimator(trajectory.getInitialPose());
    }
}
