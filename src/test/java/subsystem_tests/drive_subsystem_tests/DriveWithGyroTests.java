package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.drive_subsystem_tests.utils.NavXSim;

class DriveWithGyroTests extends DriveSubsystemTestBase {

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
  void testDriveForwardWithAngleFieldRelative() {
    Timer.delay(0.02); // let the navX thread update
    NavXSim.setConnected(true);
    NavXSim.setAngle(90);
    Timer.delay(0.1); // wait 20ms for the navX Thread to update
    runCommand(driveSubsystem.drive(0.5, 0, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(90), state.angle);
    }
  }

  @Test
  void testDriveSidewaysWithAngleFieldRelative() {
    Timer.delay(0.02); // wait 20ms for the navX Thread to update
    NavXSim.setConnected(true);
    NavXSim.setAngle(45);
    Timer.delay(0.1); // wait 20ms for the navX Thread to update
    runCommand(driveSubsystem.drive(0, 0.5, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      // 90 + 45 degrees
      assertEquals(Rotation2d.fromDegrees(90 + 45), state.angle);
    }
  }
}
