package command_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static subsystem_tests.led_tests.utils.LEDTestUtils.checkForColorInAll;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.IntakeWithLEDCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BasicIntakeTest extends CommandTestBase {
  private IntakeSubsystem intakeSubsystem;

  private IntakeWithLEDCommand intakeCommand;

  @BeforeEach
  public void setUp() {
    super.setUp();
    intakeSubsystem = new IntakeSubsystem();
    intakeCommand = new IntakeWithLEDCommand(intakeSubsystem);

    commandScheduler.schedule(intakeCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
    intakeSubsystem.close();
  }

  @Test
  void testSpeed() {
    commandScheduler.run();
    // Motor shouldn't be 0 when intaking
    assertNotEquals(0, intakeSubsystem.getArmIntakeSpeed());
  }

  @Test
  void testLEDBlinking() {
    commandScheduler.run();
    Timer.delay(0.2); // let led loop do its thing
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should be red when started");
    Timer.delay(0.2);
    checkForColorInAll(ledSubsystem, Color.kBlack, "Color should be closed when blinking red");
  }

  @Test
  void testCommandEnds() {
    commandScheduler.run();
    ColorSensorV3Wrapped.setNoteColor(true);
    commandScheduler.run();
    assertEquals(true, intakeCommand.isFinished(), "Command should be finished when ending");
    assertEquals(0, intakeSubsystem.getArmIntakeSpeed(), "Motor should stop after detecting color");
  }
}
