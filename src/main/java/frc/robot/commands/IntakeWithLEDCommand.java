package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.led_commands.LEDBlinkRedCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSystem;

public class IntakeWithLEDCommand extends ParallelRaceGroup {
  public IntakeWithLEDCommand(IntakeSubsystem intakeSubsystem) {
    super(intakeSubsystem.run(), new LEDBlinkRedCommand(LEDSystem.getInstance()));
  }
}
