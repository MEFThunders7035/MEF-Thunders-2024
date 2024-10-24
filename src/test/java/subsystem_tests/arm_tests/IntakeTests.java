package subsystem_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.SubsystemTestBase;

class IntakeTests extends SubsystemTestBase {
  private IntakeSubsystem intakeSubsystem;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
    intakeSubsystem = new IntakeSubsystem();
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
    intakeSubsystem.close();
  }

  @Test
  void testIntakeSubsystem() {
    runCommand(intakeSubsystem.run(0.5));

    assertEquals(0.5, intakeSubsystem.getArmIntakeSpeed(), 0.001);
  }

  @Test
  void testIntakeNoteDetected() {
    ColorSensorV3Wrapped.setNoteColor(true);
    assertEquals(true, intakeSubsystem.hasNote(), "Intake should detect a note");
    ColorSensorV3Wrapped.setNoteColor(false);
    assertEquals(false, intakeSubsystem.hasNote(), "Intake should not detect a note");
    // TODO: Add more tests for different RGBD values
  }

  @Test
  void testIntakeStopsOnNote() {
    ColorSensorV3Wrapped.setNoteColor(false);
    runCommand(intakeSubsystem.run(0.5));
    assertEquals(
        0.5,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should be running when a note is not detected");
    ColorSensorV3Wrapped.setNoteColor(true);
    commandScheduler.run();
    assertEquals(
        0,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should stop when a note is detected");
  }

  @Test
  void testIntakeForcePushes() {
    ColorSensorV3Wrapped.setNoteColor(true);
    runCommand(intakeSubsystem.loadToShooter()); // force push note out
    assertEquals(
        IntakeConstants.kArmIntakeRunSpeed,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Arm Intake Motor should run when loading to shooter");
    assertEquals(
        0,
        intakeSubsystem.getGroundIntakeSpeed(),
        0.001,
        "Ground Intake Motor shouldn't run when loading to shooter");
    commandScheduler.run();
    assertEquals(
        IntakeConstants.kArmIntakeRunSpeed,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should continue to run when forced");
  }

  @Test
  void testIntakeSubsystemWithNote() {
    ColorSensorV3Wrapped.setNoteColor(true);
    runCommand(intakeSubsystem.run(0.5));
    assertEquals(
        0,
        intakeSubsystem.getArmIntakeSpeed(),
        0.001,
        "Intake motor should not be running when a note is detected");
  }
}
