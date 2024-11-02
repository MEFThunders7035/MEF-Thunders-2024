package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.RotationPIDController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommands {
  /**
   * A command that handles all boost and deadband calculations for the drive command. It also
   * handles the inversion of the controller, so be careful when using this command.
   *
   * <p><b>Warning</b>: This command meant to be used with the <b>Logitech</b> controller only.
   * Xbox/playstation controllers won't work due to forward and backward being inverted.
   *
   * @param driveSubsystem DriveSubsystem
   * @param controller Logitech controller
   * @return The command to drive the robot with the <b>Logitech</b> controller
   */
  public static Command driveWithController(
      DriveSubsystem driveSubsystem, XboxController controller) {
    return driveSubsystem.drive(
        () ->
            addBoost(
                -controller.getLeftY(),
                getBoostByValue(
                    controller)), // this is inverted because logitech controller is inverted
        () ->
            addBoost(
                -controller.getLeftX(),
                getBoostByValue(controller)), // ! THIS IS INVERTED BECAUSE +y IS LEFT of robot
        () ->
            addBoost(
                -controller.getRightX(),
                getBoostByValue(controller)) // ! THIS IS INVERTED BECAUSE + IS COUNTERCLOCKWISE),
        );
  }

  public static Command driveFacingShooter(
      DriveSubsystem driveSubsystem, XboxController controller) {
    var rotController = getPIDController();
    return driveSubsystem.drive(
        () ->
            addBoost(
                -controller.getLeftY(),
                getBoostByValue(
                    controller)), // this is inverted because logitech controller is inverted
        () ->
            addBoost(
                -controller.getLeftX(),
                getBoostByValue(controller)), // ! THIS IS INVERTED BECAUSE +y IS LEFT of
        () ->
            rotController.calculate(driveSubsystem.getRotationDifferenceToShooter().getRadians()));
  }

  private static double addBoost(double value, double boost) {
    /*
     * we wish to add the DriveSensitivity amount of the value as boost
     * so calculation is:
     *
     * (1 - boost) * value + boost * value
     *
     * when we factor for "value" we get:
     *
     * value * ( driveSensitivity * (1 - boost) +  boost)
     *
     */
    final var boostMultiplier = OIConstants.kDriveSensitivity * (1 - boost) + boost;
    return MathUtil.applyDeadband(value * boostMultiplier, OIConstants.kDriveDeadband);
  }

  /**
   * Get the boost value for the drive command Boosts According to the shoulder analog inputs
   * doesn't go less than 0.2 for the convince of the driver
   *
   * @param controller
   * @return a double value between -0.8 and 1.0
   */
  private static double getBoostByValue(XboxController controller) {
    var boostBy = controller.getRightTriggerAxis();
    var reduceAmount = controller.getLeftTriggerAxis(); // between 0 and 1

    double totalBoost = boostBy - reduceAmount;

    totalBoost = MathUtil.clamp(totalBoost, OIConstants.kMinBoost, OIConstants.kMaxBoost);
    return totalBoost;
  }

  private static PIDController getPIDController() {
    var controller =
        new PIDController(
            RotationPIDController.kP, RotationPIDController.kI, RotationPIDController.kD);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    return controller;
  }
}
