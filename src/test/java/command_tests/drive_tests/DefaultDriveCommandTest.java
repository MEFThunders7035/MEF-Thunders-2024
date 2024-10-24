package command_tests.drive_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.drive_subsystem_tests.utils.DriveTestUtils;

class DefaultDriveCommandTest extends CommandTestBase {
  DriveSubsystem driveSubsystem;
  XboxController controller;
  XboxControllerSim controllerSim;
  Command defaultDriveCommand;

  @BeforeEach
  public void setUp() {
    super.setUp();
    driveSubsystem = new DriveSubsystem();
    controller = new XboxController(0);
    controllerSim = new XboxControllerSim(controller);
    defaultDriveCommand = DriveCommands.driveWithController(driveSubsystem, controller);
    commandScheduler.schedule(defaultDriveCommand);
  }

  @AfterEach
  public void tearDown() {
    driveSubsystem.close();
    resetController();
    super.tearDown();
  }

  /** Sets all axis to 0 */
  void resetController() {
    for (int i = 0; i < controller.getAxisCount(); i++) {
      controllerSim.setRawAxis(i, 0);
    }
  }

  @Test
  void testDefaultDriveCommandForward() {
    commandScheduler.run();

    assertTrue(commandScheduler.isScheduled(defaultDriveCommand));

    controllerSim.setLeftY(1);
    DriverStationSim.notifyNewData();
    assertEquals(1, controller.getLeftY(), 0.001);
    commandScheduler.run();
    SimHooks.stepTiming(2); // ! This is for the rate limiter not to kick in
    commandScheduler.run();

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      // boost is 0 so speed should be timed with sensitivity
      assertEquals(
          DriveConstants.kMaxSpeedMetersPerSecond * OIConstants.kDriveSensitivity,
          state.speedMetersPerSecond,
          0.05);
      assertEquals(Rotation2d.fromDegrees(0), state.angle);
    }
  }

  @Test
  void testDefaultDriveCommandSideways() {
    commandScheduler.run();

    assertTrue(commandScheduler.isScheduled(defaultDriveCommand));

    controllerSim.setLeftX(1);
    DriverStationSim.notifyNewData();
    assertEquals(1, controller.getLeftX(), 0.001);
    commandScheduler.run();
    SimHooks.stepTiming(2); // ! This is for the rate limiter not to kick in
    commandScheduler.run();

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      // boost is 0 so speed should be timed with sensitivity
      assertEquals(
          DriveConstants.kMaxSpeedMetersPerSecond * OIConstants.kDriveSensitivity,
          state.speedMetersPerSecond,
          0.05);
      // -90 because +y is inverted in the field coordinate system
      assertEquals(Rotation2d.fromDegrees(-90), state.angle);
    }
  }

  @Test
  void testDefaultDriveCommandBoost() {
    commandScheduler.run();

    assertTrue(commandScheduler.isScheduled(defaultDriveCommand));

    controllerSim.setLeftY(1);
    controllerSim.setRightTriggerAxis(1);
    DriverStationSim.notifyNewData();
    assertEquals(1, controller.getLeftY(), 0.001);
    commandScheduler.run();
    SimHooks.stepTiming(2); // ! This is for the rate limiter not to kick in
    commandScheduler.run();

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      // boost is 1 so speed should be max speed
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, state.speedMetersPerSecond, 0.05);
      assertEquals(Rotation2d.fromDegrees(0), state.angle);
    }
  }

  @Test
  void testDefaultDriveCommandRotate() {
    commandScheduler.run();

    assertTrue(commandScheduler.isScheduled(defaultDriveCommand));

    controllerSim.setRightX(1);
    controllerSim.setRightTriggerAxis(0);
    DriverStationSim.notifyNewData();
    assertEquals(1, controller.getRightX(), 0.001);
    commandScheduler.run();
    SimHooks.stepTiming(2); // ! This is for the rate limiter not to kick in
    commandScheduler.run();

    // boost is 0 so speed should be timed with sensitivity
    // deadband is 0.02 which increases all values by 0.02
    // ! +rot is counter clockwise in field coordinate system, so it should be negative
    var shouldBe =
        DriveTestUtils.driveToChassisSpeeds(
            0, 0, MathUtil.applyDeadband(-1 * OIConstants.kDriveSensitivity, 0.02));
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }
}
