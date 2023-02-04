
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double translationStickDeadband = 0.1;
    public static final double rotationStickDeadband = 0.2;

   public static final int pigeonID = 30;
   public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10; // Drive Motor Controller - [FLD]
      public static final int angleMotorID = 11; // Angle Motor Controller - [FLA]
      public static final int canCoderID = 12; // Front Left Encoder - [FLE]
      public static final boolean driveMotorInvert = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(269.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 13; // Drive Motor Controller - [FRD]
      public static final int angleMotorID = 14; // Angle Motor Controller - [FRA]
      public static final int canCoderID = 15; // Front Right Encoder - [FRE]
      public static final boolean driveMotorInvert = true;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(390.8);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 16; // Drive Motor Controller - [BLD]
      public static final int angleMotorID = 17; // Angle Motor Controller - [BLA]
      public static final int canCoderID = 18; // Back Left Encoder - [BLE]
      public static final boolean driveMotorInvert = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(11.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 19; // Drive Motor Controller - [BRD]
      public static final int angleMotorID = 20; // Angle Motor Controller - [BRA]
      public static final int canCoderID = 21; // Back Right Encoder - [BRE]
      public static final boolean driveMotorInvert = true;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.9);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveMotorInvert);
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
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class PivotConstants {
    public static final int pivotMotorID = 40;
    public static final int smartCurrentLimit = 30;
    public static final double speedForward = 10;
    public static final double speedBack = -10;
    public static final double speedStop = 0;
    public static final double positionHorizontal = 0.0;
    public static final double positionPickup = -16.1427;
    public static final double positionScoring = 9.3333;
    public static final double positionTravel = 34.214;
    public static final double pivotKP = 0.025;
    public static final double pivotKI = 0.00;
    public static final double pivotKD = 0.00;
    public static final double pivotKFF = 0.00;
  }

  public static final class TelescopicConstants {
    public static final int telescopicMotorID = 40; // Falcon motor ID
    public static final int smartCurrentLimit = 30;
    public static final double telescopicSpeedForward = 5;
    public static final double telescopicSpeedBack = -5;
    public static final double telescopicStop = 0;
    //
    public static final double telescopicForward = 0.0;
    public static final double telescopicBack = -10.0;
    public static final double telescopicTravel = 30.0;
    //
    public static final double telescopicKP = 0.6;
    public static final double telescopicKI = 0.00;
    public static final double telescopicKD = 0.00;
    public static final double telescopicKFF = 0.00;
    //
    public static final double kForwardPercent = 0.15;
    public static final double kReversePercent = 0.15;
  }
}
