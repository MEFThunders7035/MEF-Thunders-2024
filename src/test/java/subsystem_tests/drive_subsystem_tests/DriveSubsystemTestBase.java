package subsystem_tests.drive_subsystem_tests;

import frc.robot.subsystems.DriveSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import subsystem_tests.SubsystemTestBase;

public class DriveSubsystemTestBase extends SubsystemTestBase {
  protected DriveSubsystem driveSubsystem;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
    driveSubsystem = new DriveSubsystem();
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
    driveSubsystem.close();
  }
}
