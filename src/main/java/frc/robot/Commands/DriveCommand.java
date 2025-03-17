// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Util.Constants.constants_Sim;

import java.util.function.DoubleSupplier;

public class DriveCommand {
  private static final double DEADBAND = 0.085;
  // private static final double ANGLE_KP = 5.0;
  // private static final double ANGLE_KD = 0.4;
  // private static final double ANGLE_MAX_VELOCITY = 8.0;
  // private static final double ANGLE_MAX_ACCELERATION = 20.0;
//   private static final double FF_START_DELAY = 2.0; // Secs
//   private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  // private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  // private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommand() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative s_Drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive s_Drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * (constants_Sim.currentMode==constants_Sim.simMode? constants_Sim.SIM_MAX_SPEED_METERS_PER_SEC:s_Drive.getMaxLinearSpeedMetersPerSec()),
                  linearVelocity.getY() * (constants_Sim.currentMode==constants_Sim.simMode? constants_Sim.SIM_MAX_SPEED_METERS_PER_SEC:s_Drive.getMaxLinearSpeedMetersPerSec()),
                  omega * (constants_Sim.currentMode==constants_Sim.simMode? constants_Sim.SIM_MAX_ANGULAR_SPEED_RadPS:s_Drive.getMaxAngularSpeedRadPerSec()));
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          s_Drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? s_Drive.getRotation().plus(new Rotation2d(Math.PI))
                      : s_Drive.getRotation()));
        },
        s_Drive);
  }


  public static Command driveReef(Drive s_Drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier)
  {
    return Commands.run(
      () ->
      {
          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble(),
                  ySupplier.getAsDouble(),
                  thetaSupplier.getAsDouble());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          s_Drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? s_Drive.getRotation().plus(new Rotation2d(Math.PI))
                      : s_Drive.getRotation()));
        },
        s_Drive);
  }


  // /**
  //  * Field relative s_Drive command using joystick for linear control and PID for angular control.
  //  * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
  //  * absolute rotation with a joystick.
  //  */
  // public static Command joystickDriveAtAngle(
  //     Drive s_Drive,
  //     DoubleSupplier xSupplier,
  //     DoubleSupplier ySupplier,
  //     Supplier<Rotation2d> rotationSupplier) {

  //   // Create PID controller
  //   ProfiledPIDController angleController =
  //       new ProfiledPIDController(
  //           ANGLE_KP,
  //           0.0,
  //           ANGLE_KD,
  //           new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
  //   angleController.enableContinuousInput(-Math.PI, Math.PI);

  //   // Construct command
  //   return Commands.run(
  //           () -> {
  //             // Get linear velocity
  //             Translation2d linearVelocity =
  //                 getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

  //             // Calculate angular speed
  //             double omega =
  //                 angleController.calculate(
  //                     s_Drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

  //             // Convert to field relative speeds & send command
  //             ChassisSpeeds speeds =
  //                 new ChassisSpeeds(
  //                     linearVelocity.getX() * s_Drive.getMaxLinearSpeedMetersPerSec(),
  //                     linearVelocity.getY() * s_Drive.getMaxLinearSpeedMetersPerSec(),
  //                     omega);
  //             boolean isFlipped =
  //                 DriverStation.getAlliance().isPresent()
  //                     && DriverStation.getAlliance().get() == Alliance.Red;
  //             s_Drive.runVelocity(
  //                 ChassisSpeeds.fromFieldRelativeSpeeds(
  //                     speeds,
  //                     isFlipped
  //                         ? s_Drive.getRotation().plus(new Rotation2d(Math.PI))
  //                         : s_Drive.getRotation()));
  //           },
  //           s_Drive)

  //       // Reset PID controller when command starts
  //       .beforeStarting(() -> angleController.reset(s_Drive.getRotation().getRadians()));
  // }

  /**
   * Measures the velocity feedforward constants for the s_Drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  // public static Command feedforwardCharacterization(Drive s_Drive) {
  //   List<Double> velocitySamples = new LinkedList<>();
  //   List<Double> voltageSamples = new LinkedList<>();
  //   Timer timer = new Timer();

  //   return Commands.sequence(
  //       // Reset data
  //       Commands.runOnce(
  //           () -> {
  //             velocitySamples.clear();
  //             voltageSamples.clear();
  //           }),

  //       // Allow modules to orient
  //       Commands.run(
  //               () -> {
  //                 s_Drive.runCharacterization(0.0);
  //               },
  //               s_Drive)
  //           .withTimeout(FF_START_DELAY),

  //       // Start timer
  //       Commands.runOnce(timer::restart),

  //       // Accelerate and gather data
  //       Commands.run(
  //               () -> {
  //                 double voltage = timer.get() * FF_RAMP_RATE;
  //                 s_Drive.runCharacterization(voltage);
  //                 velocitySamples.add(s_Drive.getFFCharacterizationVelocity());
  //                 voltageSamples.add(voltage);
  //               },
  //               s_Drive)

  //           // When cancelled, calculate and print results
  //           .finallyDo(
  //               () -> {
  //                 int n = velocitySamples.size();
  //                 double sumX = 0.0;
  //                 double sumY = 0.0;
  //                 double sumXY = 0.0;
  //                 double sumX2 = 0.0;
  //                 for (int i = 0; i < n; i++) {
  //                   sumX += velocitySamples.get(i);
  //                   sumY += voltageSamples.get(i);
  //                   sumXY += velocitySamples.get(i) * voltageSamples.get(i);
  //                   sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
  //                 }
  //                 double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
  //                 double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

  //                 NumberFormat formatter = new DecimalFormat("#0.00000");
  //                 System.out.println("********** Drive FF Characterization Results **********");
  //                 System.out.println("\tkS: " + formatter.format(kS));
  //                 System.out.println("\tkV: " + formatter.format(kV));
  //               }));
  // }

  // /** Measures the robot's wheel radius by spinning in a circle. */
  // public static Command wheelRadiusCharacterization(Drive s_Drive) {
  //   SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
  //   WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

  //   return Commands.parallel(
  //       // Drive control sequence
  //       Commands.sequence(
  //           // Reset acceleration limiter
  //           Commands.runOnce(
  //               () -> {
  //                 limiter.reset(0.0);
  //               }),

  //           // Turn in place, accelerating up to full speed
  //           Commands.run(
  //               () -> {
  //                 double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
  //                 s_Drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
  //               },
  //               s_Drive)),

  //       // Measurement sequence
  //       Commands.sequence(
  //           // Wait for modules to fully orient before starting measurement
  //           Commands.waitSeconds(1.0),

  //           // Record starting measurement
  //           Commands.runOnce(
  //               () -> {
  //                 state.positions = s_Drive.getWheelRadiusCharacterizationPositions();
  //                 state.lastAngle = s_Drive.getRotation();
  //                 state.gyroDelta = 0.0;
  //               }),

  //           // Update gyro delta
  //           Commands.run(
  //                   () -> {
  //                     var rotation = s_Drive.getRotation();
  //                     state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
  //                     state.lastAngle = rotation;
  //                   })

  //               // When cancelled, calculate and print results
  //               .finallyDo(
  //                   () -> {
  //                     double[] positions = s_Drive.getWheelRadiusCharacterizationPositions();
  //                     double wheelDelta = 0.0;
  //                     for (int i = 0; i < 4; i++) {
  //                       wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
  //                     }
  //                     double wheelRadius =
  //                         (state.gyroDelta * DriveConstants.s_DriveBaseRadius) / wheelDelta;

  //                     NumberFormat formatter = new DecimalFormat("#0.000");
  //                     System.out.println(
  //                         "********** Wheel Radius Characterization Results **********");
  //                     System.out.println(
  //                         "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
  //                     System.out.println(
  //                         "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
  //                     System.out.println(
  //                         "\tWheel Radius: "
  //                             + formatter.format(wheelRadius)
  //                             + " meters, "
  //                             + formatter.format(Units.metersToInches(wheelRadius))
  //                             + " inches");
  //                   })));
  // }

  // private static class WheelRadiusCharacterizationState {
  //   double[] positions = new double[4];
  //   Rotation2d lastAngle = new Rotation2d();
  //   double gyroDelta = 0.0;
  // }
}
