/* BSD 3-Clause License

Copyright (c) 2022, FRC Team 3512
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import frc.robot.Constants.VisionConstants;
import frc.robot.autos.*;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDrivePoseEstimator swervePoseEstimator;
    private SwerveModule[] mSwerveMods;

    private Field2d field;

    public PhotonCameraWrapper pcw1;
    public PhotonCameraWrapper pcw2;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.kPigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        pcw1 = new PhotonCameraWrapper(VisionConstants.kCameraName1, VisionConstants.kRobotToCam1);
        pcw2 = new PhotonCameraWrapper(VisionConstants.kCameraName2, VisionConstants.kRobotToCam2);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getOdometryPosition(),
                        mSwerveMods[1].getOdometryPosition(),
                        mSwerveMods[2].getOdometryPosition(),
                        mSwerveMods[3].getOdometryPosition()
                },
                new Pose2d());

        field = new Field2d();
        field.getObject("Auto path").setTrajectory(DropAndDriveAndStrafeAndPickup.strafePath_step1);
        

        ShuffleboardTab tab = Shuffleboard.getTab("Swerve Drive");
        tab.add("Field", field);
        for (SwerveModule mod : mSwerveMods) {
            tab.addString("Mod " + mod.moduleNumber + " Position", () -> mod.getPosition().toString());
        }
        tab.addString("Pose", () -> getPose().toString());
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // private Rotation2d inverse(Rotation2d a) {
    // return new Rotation2d(a.getCos(), -a.getSin());
    // }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void resetPoseAndGyro() {
        zeroGyro();
        resetPoseEstimator();
    }

    public void resetPoseEstimator() {
        swervePoseEstimator.resetPosition(getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getOdometryPosition(),
                        mSwerveMods[1].getOdometryPosition(),
                        mSwerveMods[2].getOdometryPosition(),
                        mSwerveMods[3].getOdometryPosition()
                },
                getPose());
    }

    public void setPoseEstimator(Pose2d pose) {
        swervePoseEstimator.resetPosition(getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getOdometryPosition(),
                        mSwerveMods[1].getOdometryPosition(),
                        mSwerveMods[2].getOdometryPosition(),
                        mSwerveMods[3].getOdometryPosition()
                },
                pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.kInvertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getRawPitch() {
        return gyro.getPitch();
    }

    @Override
    public void periodic() {
        swervePoseEstimator.update(getYaw(),
                new SwerveModulePosition[] {
                        mSwerveMods[0].getOdometryPosition(),
                        mSwerveMods[1].getOdometryPosition(),
                        mSwerveMods[2].getOdometryPosition(),
                        mSwerveMods[3].getOdometryPosition()
                }); // get the rotation and offset for encoder

        if(!DriverStation.isAutonomousEnabled()){

            Optional<EstimatedRobotPose> result = pcw1.getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition());

            if (result.isPresent()) {
                EstimatedRobotPose camPose1 = result.get();
                swervePoseEstimator.addVisionMeasurement(
                        camPose1.estimatedPose.toPose2d(), camPose1.timestampSeconds);
                field.getObject("Cam Est Pos 1").setPose(camPose1.estimatedPose.toPose2d());
            } else {
                // move it way off the screen to make it disappear
                field.getObject("Cam Est Pos 1").setPose(new Pose2d(-100, -100, new Rotation2d()));
            }

            result = pcw2.getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition());

            if (result.isPresent()) {
                EstimatedRobotPose camPose2 = result.get();
                swervePoseEstimator.addVisionMeasurement(
                        camPose2.estimatedPose.toPose2d(), camPose2.timestampSeconds);
                field.getObject("Cam Est Pos 2").setPose(camPose2.estimatedPose.toPose2d());
            } else {
                // move it way off the screen to make it disappear
                field.getObject("Cam Est Pos 2").setPose(new Pose2d(-100, -100, new Rotation2d()));
            }

            // System.out.println(swervePoseEstimator.getEstimatedPosition().getTranslation());
        }

        field.setRobotPose(getPose());
    }
}
