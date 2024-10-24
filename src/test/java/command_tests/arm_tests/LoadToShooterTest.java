package command_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LoadToShooterTest extends CommandTestBase {
  IntakeSubsystem intakeSubsystem;
  Command command;

  @BeforeEach
  public void setUp() {
    super.setUp();

    intakeSubsystem = new IntakeSubsystem();
    command = intakeSubsystem.loadToShooter();

    commandScheduler.schedule(command);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
    intakeSubsystem.close();
  }

  @Test
  void testPushesOutNoteAndStops() {
    ColorSensorV3Wrapped.setNoteColor(true);
    commandScheduler.run();
    SimHooks.stepTiming(0.6); // handle "waitANDCondition" s wait part
    commandScheduler.run();

    ColorSensorV3Wrapped.setNoteColor(false);
    commandScheduler.run();
    commandScheduler.run();

    assertEquals(
        false,
        command.isScheduled(),
        "command should finish after note isn't detected AND 0.5 seconds elapses");
  }

  @Test
  void testItDoesntStopBeforeTime() {
    ColorSensorV3Wrapped.setNoteColor(true);
    commandScheduler.run();

    SimHooks.stepTiming(0.1); // not enough time for it to stop
    commandScheduler.run();

    ColorSensorV3Wrapped.setNoteColor(false);
    commandScheduler.run();
    commandScheduler.run();

    assertEquals(true, command.isScheduled(), "command shouldn't finish before the time");
  }

  /** This is for the fact that, if somehow the color sensor fails, that might cause issues. */
  @Test
  void testItRunsWithoutDetection() {
    ColorSensorV3Wrapped.setNoteColor(false);
    commandScheduler.run();
    commandScheduler.run();

    assertEquals(
        true,
        command.isScheduled(), //
        "command should run even if no note cannot be detected");

    SimHooks.stepTiming(0.6);

    commandScheduler.run();
    commandScheduler.run();

    assertEquals(
        false,
        command.isScheduled(),
        "command should stop after 0.5 second time, when no note is detected");
  }

  @Test
  void testItActuallyRuns() {
    ColorSensorV3Wrapped.setNoteColor(false);
    commandScheduler.run();
    commandScheduler.run();

    assertNotEquals(0, intakeSubsystem.getArmIntakeSpeed());

    SimHooks.stepTiming(0.6);
    commandScheduler.run();
    commandScheduler.run();

    assertEquals(0, intakeSubsystem.getArmIntakeSpeed());
  }
}
