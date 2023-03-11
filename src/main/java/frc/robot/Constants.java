
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

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

    /* Solenoid constants */
    public static final int kSolenoidCanID = 7;

    /* Generic/Default constants */
    public static final double kDefaultVoltage = 12.0;

    public static final class Swerve {
        public static final double kTranslationStickDeadband = 0.1;
        public static final double kRotationStickDeadband = 0.4;

        public static final int kPigeonID = 30;
        public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double kTrackWidth = Units.inchesToMeters(17.75);
        public static final double kWheelBase = Units.inchesToMeters(17.75);
        public static final double kWheelDiameter = Units.inchesToMeters(4.0);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        public static final double kOpenLoopRamp = 0.25;
        public static final double kClosedLoopRamp = 0.0;

        public static final double kDriveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double kAngleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double kVoltageComp = kDefaultVoltage;

        /* Swerve Current Limiting */
        public static final int kAngleContinuousCurrentLimit = 20;
        public static final int kDriveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double kAngleKP = 0.01;
        public static final double kAngleKI = 0.0;
        public static final double kAngleKD = 0.0;
        public static final double kAngleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double kDriveKP = 0.1;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double kDriveKS = 0.667;
        public static final double kDriveKV = 2.44;
        public static final double kDriveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double kDriveConversionPositionFactor = (kWheelDiameter * Math.PI) / kDriveGearRatio;
        public static final double kDriveConversionVelocityFactor = kDriveConversionPositionFactor / 60.0;
        public static final double kAngleConversionFactor = 360.0 / kAngleGearRatio;

        /* Swerve Profiling Values */
        public static final double kMaxSpeed = 4.5; // meters per second
        public static final double kMaxAngularVelocityInRadiansPerSecond = 4 * Math.PI;

        /* Neutral Modes */
        public static final IdleMode kAngleNeutralMode = IdleMode.kBrake;
        public static final IdleMode kDriveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean kDriveInvert = false;
        public static final boolean kAngleInvert = false;

        /* Angle Encoder Invert */
        public static final boolean kCanCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int kDriveMotorID = 10; // Drive Motor Controller - [FLD]
            public static final int kAngleMotorID = 11; // Angle Motor Controller - [FLA]
            public static final int kCanCoderID = 12; // Front Left Encoder - [FLE]
            public static final boolean kDriveMotorInvert = true;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(280.3);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset, kDriveMotorInvert);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int kDriveMotorID = 13; // Drive Motor Controller - [FRD]
            public static final int kAngleMotorID = 14; // Angle Motor Controller - [FRA]
            public static final int kCanCoderID = 15; // Front Right Encoder - [FRE]
            public static final boolean kDriveMotorInvert = true;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(241.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset, kDriveMotorInvert);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int kDriveMotorID = 16; // Drive Motor Controller - [BLD]
            public static final int kAngleMotorID = 17; // Angle Motor Controller - [BLA]
            public static final int kCanCoderID = 18; // Back Left Encoder - [BLE]
            public static final boolean kDriveMotorInvert = true;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(212.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    kCanCoderID, kAngleOffset, kDriveMotorInvert);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int kDriveMotorID = 19; // Drive Motor Controller - [BRD]
            public static final int kAngleMotorID = 20; // Angle Motor Controller - [BRA]
            public static final int canCoderID = 21; // Back Right Encoder - [BRE]
            public static final boolean kDriveMotorInvert = true;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(268.5); // old 280.3
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID,
                    kAngleMotorID,
                    canCoderID, kAngleOffset, kDriveMotorInvert);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final double kBalanceSpeed = 0.3;

        public static TrajectoryConfig forwardConfig = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Swerve.swerveKinematics).setReversed(false);
        public static TrajectoryConfig reverseConfig = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Swerve.swerveKinematics).setReversed(true);
        public static TrajectoryConfig balanceForwardConfig = new TrajectoryConfig(
                kBalanceSpeed,
                kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Swerve.swerveKinematics).setReversed(false);
        public static TrajectoryConfig balanceReverseConfig = new TrajectoryConfig(
                kBalanceSpeed,
                kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Swerve.swerveKinematics).setReversed(true);
    }

    public static final class PivotConstants {
        public static final int kPivotMotorID = 40;
        public static final int kSmartCurrentLimit = 30;
        public static final int kEncoderChannel = 1;
        public static final double kPivotAllowableError = 2;
        public static final double kNominalVoltage = kDefaultVoltage;

        public static final TrapezoidProfile.Constraints kProfileConstraints = new TrapezoidProfile.Constraints(
                200, 125);

        // Degrees. Adding this to the encoder reading should give 0 when
        // the arm is vertical.
        public static final double kPivotVerticalOffset = -183;

        // Setpoints. All setpoints given in degrees from vertical.
        public static final double kPositionPickupCone = 129;
        public static final double kPositionScoringRelease = 60; // release angle
        public static final double kPositionScoringCone = 50; // scoring angle
        public static final double kPositionScoringCube = -69;
        public static final double kPositionPickupCube = 132;
        public static final double kPositionTravel = 0;
    }

    public static final class TelescopicConstants {
        public static final int kTelescopicMotorID = 45; // Falcon motor ID
        public static final int kCurrentLimit = 30;
        public static final int kTriggerThresholdCurrent = 40;
        public static final double kTriggerThresholdTime = 1.5;
        public static final double kTelescopicSpeedForward = 5;
        public static final double kTelescopicSpeedBack = -5;
        public static final double kTelescopicStop = 0;
        public static final int kTimeoutMs = 0;

        public static final double kPeakOutputForwardPercent = 12;
        public static final double kPeakOutputReversePercent = -12;

        public static final double kTelescopicFar = 70500;
        public static final double kTelescopicRetracted = 0;
        public static final double kTelescopicMid = 34940;

        public static final double kTelescopicKP = 0.07;
        public static final double kTelescopicKI = 0.00;
        public static final double kTelescopicKD = 0.04;
        public static final double kTelescopicKFF = 0.00;
        public static final int kTelescopicSlotIdxKP = 0;
        public static final int kTelescopicSlotIdxKI = 0;
        public static final int kTelescopicSlotIdxKD = 0;
        public static final int kTelescopicSlotIdxKFF = 0;

        public static final double kClosedLoopPeakOutputPercent = 0.5;
        public static final int kClosedLoopPeakOutputSlotIdx = 0;
        public static final int kSelectedFeedbackSensorPidIdx = 0;
        public static final double kAllowableClosedloopError = 100;
        public static final int kAllowableClosedloopErrorSlotIdx = 0;

        public static final int kPrimaryClosedLoopErrorPidx = 0;
    }

    public static final class GripperConstants {
        public static final int kGripperForwardChannel = 1;
        public static final int kGripperReverseChannel = 0;
        public static final double kGripperReleaseTimeout = 0.5;
    }

    public static final class IntakeConstants {
        public static final int kIntakeRightMotorID = 55;
        public static final int kIntakeLeftMotorID = 60;
        public static final boolean kIntakeRightMotorInverted = true;
        public static final boolean kIntakeLeftMotorInverted = false;
        public static final double kIntakeMotorPercentExtend = 1.0;
        public static final double kIntakeMotorPercentRetract = -kIntakeMotorPercentExtend;
        public static final double kIntakeExtendTimeout = 0.75;
        public static final double kIntakeRetractTimeout = kIntakeExtendTimeout;
        public static final int kSpinningIntakeMotorID = 62; // Falcon motor ID
        public static final boolean kSpinningIntakeMotorInverted = false;
        public static final double kIntakeSpinningMotorForward = 0.5;
        public static final double kIntakeSpinningMotorOff = 0;
        public static final int kSmartCurrentLimit = 10;
        public static final double kStoppedSpeed = 0;

        public static final double kIntakePositionRightRetracted = 0.07;
        public static final double kIntakePositionRightPickup = 14.52;
        public static final double kIntakePositionRightExtended = 16.00;
        public static final double kIntakePositionLeftRetracted = 0.07;
        public static final double kIntakePositionLeftPickup = 14.76;
        public static final double kIntakePositionLeftExtended = 16.25;
        public static final double kPositionConversionFactor = 1;

        public static final double kIntakeKP = 0.0002;
        public static final double kIntakeKI = 0.00;
        public static final double kIntakeKD = 0.00;
        public static final double kIntakeKFF = 0.00015;

        public static final double kSmartMotionMaxVelocity = 1200;
        public static final double kSmartMotionMaxAccel = 2500;
        public static final double kSmartMotionMinOutputVelocity = 0;

        public static final double kNominalVoltage = kDefaultVoltage;

        public static final int kSlotId = 0;

        public static final double kIntakeAllowableError = 0.2;
    }

    public static final class ConeGuideConstants {
        public static final int kConeGuideForwardChannel = 2;
        public static final int kConeGuideReverseChannel = 3;
        public static final double kConeGuideRetractTimeout = 0.35;
        public static final double kConeGuideDeployTimeout = 0.5;
    }

    public static final class VisionConstants {
        public static final String kCameraName1 = "OV5647";
        public static final String kCameraName2 = "camera2";

        public static final Transform3d kRobotToCam1 = new Transform3d(
                new Translation3d(0.098, -0.038, 0.683),
                new Rotation3d(0.0, 0.0, Math.PI));

        public static final Transform3d kRobotToCam2 = new Transform3d(
                new Translation3d(0.098, -0.038, 0.683),
                new Rotation3d(0.0, 0.0, Math.PI));

        public static final double kTranslationFF = 0.007;
        public static final double kStrafeFF = kTranslationFF;
    }
}
