package command_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MoveArmToAmpTest extends CommandTestBase {
  private Command moveArmToAmpCommand;
  private ArmSubsystem armSubsystem;

  @BeforeEach
  public void setUp() {
    super.setUp();

    armSubsystem = new ArmSubsystem();
    moveArmToAmpCommand = armSubsystem.setArmToAmp();
    commandScheduler.schedule(moveArmToAmpCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
    armSubsystem.close();
  }

  @Test
  void testArmSetpointIsTrue() {
    commandScheduler.run();
    double setpoint = SmartDashboard.getNumber("Setpoint", -1);

    // !if this value 0.5 is changed or moved to a constant instead (it should be) change this
    assertEquals(0.5, setpoint, "Arm should have been on 0.5 (meaning the amp)");
  }
}
