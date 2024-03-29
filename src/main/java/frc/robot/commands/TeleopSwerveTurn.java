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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

public class TeleopSwerveTurn extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationXSup;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
    private PIDController rotationController = new PIDController(4.0, 0, 0);

    public TeleopSwerveTurn(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationXSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationXSup = rotationXSup;

        rotationController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    public void execute() {
        // double rotationOutput =
        // rotationController.calculate(s_Swerve.getYaw().getRadians());

        /* Get Values, Deadband */
        double translationVal = translationLimiter.calculate(
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.kTranslationStickDeadband));
        double strafeVal = strafeLimiter.calculate(
                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.kTranslationStickDeadband));
        double rotationVal = rotationLimiter.calculate(
                MathUtil.applyDeadband(rotationXSup.getAsDouble(), Constants.Swerve.kRotationStickDeadband));

        /* Drive */
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.kMaxSpeed),
                rotationVal * Constants.Swerve.kMaxAngularVelocityInRadiansPerSecond,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, false);
    }
}
