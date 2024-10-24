package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveSubsystemExtrasTests extends DriveSubsystemTestBase {
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
  void testSetX() {
    runCommand(driveSubsystem.setX());
    var states = driveSubsystem.getModuleDesiredStates();
    int[] expectedAngles = {45, -45, -45, 45};

    for (int i = 0; i < expectedAngles.length; i++) {
      assertEquals(expectedAngles[i], states[i].angle.getDegrees());
    }
  }
}
