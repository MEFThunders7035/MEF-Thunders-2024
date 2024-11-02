package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ColorSensorConstants;
import frc.robot.commands.VibrateControllerCommand;
import frc.robot.commands.WaitANDConditionCommand;
import frc.utils.sim_utils.CANSparkMAXWrapped;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped armIntake;
  private final CANSparkMAXWrapped groundIntake;
  private final ColorSensorV3Wrapped colorSensor;
  private boolean isForced = false;

  public IntakeSubsystem() {
    armIntake = new CANSparkMAXWrapped(IntakeConstants.kArmIntakeMotorCanID, MotorType.kBrushless);
    groundIntake =
        new CANSparkMAXWrapped(IntakeConstants.kGroundIntakeMotorCanID, MotorType.kBrushless);
    setupIntakeMotors();
    colorSensor = new ColorSensorV3Wrapped(ColorSensorConstants.kColorSensorPort);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ColorSensor - Distance", colorSensor.getProximity());
    SmartDashboard.putNumber("ColorSensor - Red", colorSensor.getRed());
    SmartDashboard.putNumber("ColorSensor - Green", colorSensor.getGreen());
    SmartDashboard.putNumber("ColorSensor - Blue", colorSensor.getBlue());
    SmartDashboard.putNumber("ColorSensor - IR", colorSensor.getIR());
    SmartDashboard.putBoolean("Note Detected", hasNote());
    // don't check for a note here cause it breaks the unit tests SOMEHOW!
  }

  private void setupIntakeMotors() {
    armIntake.restoreFactoryDefaults();
    groundIntake.restoreFactoryDefaults();

    armIntake.setSmartCurrentLimit(20); // NEO 550 stall current is 20A
    groundIntake.setSmartCurrentLimit(20); // NEO 550 stall current is 20A
    armIntake.setInverted(true);
    groundIntake.setInverted(true);

    armIntake.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
    groundIntake.setIdleMode(IdleMode.kCoast);

    armIntake.burnFlash();
    groundIntake.burnFlash();
  }

  @Override
  public void close() {
    armIntake.close();
    groundIntake.close();
    colorSensor.close();
  }

  public boolean hasNote() {
    int red = colorSensor.getRed();
    int blue = colorSensor.getBlue();
    // If we are really close, we will decrease the threshold.
    // Proximity is inversely proportional to distance. and max of 2047 is 0 inches.
    if (colorSensor.getProximity() > 1600) {
      red = (int) (red * (1600.0 / colorSensor.getProximity()));
      blue = (int) (blue * (1600.0 / colorSensor.getProximity()));
    }

    return colorSensor.getProximity() > 300
        && red > 700
        && blue < 9000
        && red > colorSensor.getGreen();
  }

  public double getGroundIntakeSpeed() {
    return groundIntake.get();
  }

  public double getArmIntakeSpeed() {
    return armIntake.get();
  }

  public Command stop() {
    return this.runOnce(this::stopMotors);
  }

  public Command run(DoubleSupplier armSpeed, DoubleSupplier groundSpeed) {

    return this.runIntakeCommand(armSpeed, groundSpeed, () -> false).until(this::hasNote);
  }

  public Command run(double armSpeed, double groundSpeed) {
    return this.run(() -> armSpeed, () -> groundSpeed);
  }

  public Command run(DoubleSupplier speed) {
    return this.run(speed.getAsDouble(), speed.getAsDouble());
  }

  public Command run(double speed) {
    return this.run(() -> speed);
  }

  public Command run() {
    return this.run(IntakeConstants.kArmIntakeRunSpeed, IntakeConstants.kGroundIntakeRunSpeed);
  }

  /**
   * Pushes the ball to the shooter. Warning: This command will obviously not check if the ball is
   * in the intake.
   *
   * @return
   */
  public Command loadToShooter() {
    return this.runIntakeCommand(IntakeConstants.kPushToShooterSpeed, 0, true)
        .raceWith(new WaitANDConditionCommand(0.5, () -> !hasNote())); // Stop when we have a note.
  }

  public Command vibrateControllerOnNoteCommand(XboxController controller) {
    return new VibrateControllerCommand(controller, 2, 1, 0.2)
        .alongWith(new PrintCommand("Note!"))
        .beforeStarting(new WaitUntilCommand(this::hasNote));
  }

  private void stopMotors() {
    armIntake.stopMotor();
    groundIntake.stopMotor();
  }

  private Command runIntakeCommand(
      DoubleSupplier armSpeed, DoubleSupplier groundSpeed, BooleanSupplier force) {
    return this.runEnd(
        () ->
            setIntakeSpeed(armSpeed.getAsDouble(), groundSpeed.getAsDouble(), force.getAsBoolean()),
        this::stopMotors);
  }

  private Command runIntakeCommand(double armSpeed, double groundSpeed, boolean force) {
    return runIntakeCommand(() -> armSpeed, () -> groundSpeed, () -> force);
  }

  private void setIntakeSpeed(double armSpeed, double groundSpeed, boolean force) {
    isForced = force;
    if (armSpeed > 0 && hasNote() && !force) { // If we are intaking, check if we have a note.
      armIntake.set(0);
    } else {
      armIntake.set(armSpeed);
    }

    if (groundSpeed > 0 && hasNote() && !force) { // If we are intaking, check if we have a note.
      groundIntake.set(0);
      return;
    }

    groundIntake.set(groundSpeed);
  }
}
