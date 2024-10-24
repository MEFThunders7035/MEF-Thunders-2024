package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.drive_subsystem_tests.utils.DriveTestUtils;

class DriveTests extends DriveSubsystemTestBase {
  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testRotation() {
    runCommand(driveSubsystem.drive(0, 0, 0.5, false, false));

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0, 0, 0.5);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveChassisSpeed() {
    var shouldBe = new ChassisSpeeds(2, 2, 0);
    driveSubsystem.driveRobotRelative(shouldBe);

    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveForwardRobotRelative() {
    runCommand(driveSubsystem.drive(0.5, 0, 0, false, false));

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0.5, 0, 0);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveSidewaysRobotRelative() {
    runCommand(driveSubsystem.drive(0, 0.5, 0, false, false));

    var shouldBe = DriveTestUtils.driveToChassisSpeeds(0, 0.5, 0);
    var currentlyIs = DriveTestUtils.getDesiredChassisSpeeds(driveSubsystem);

    DriveTestUtils.checkIfEqual(shouldBe, currentlyIs);
  }

  @Test
  void testDriveForwardFieldRelative() {
    runCommand(driveSubsystem.drive(0.5, 0, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(0), state.angle);
    }
  }

  @Test
  void testDriveSidewaysFieldRelative() {
    runCommand(driveSubsystem.drive(0, 0.5, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(90), state.angle);
    }
  }

  @Test
  void testDriveDiagonalFieldRelative() {
    runCommand(driveSubsystem.drive(1, 1, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(45), state.angle);
    }
  }

  @Test
  void testDriveDiagonalBackwardsFieldRelative() {
    runCommand(driveSubsystem.drive(-1, -1, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(225), state.angle);
    }
  }
}
