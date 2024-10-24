package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends ParallelCommandGroup {
  public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, XboxController controller) {
    super(
        new IntakeWithLEDCommand(intakeSubsystem),
        intakeSubsystem.vibrateControllerOnNoteCommand(controller));
  }
}
