package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  private final Spark shooterMotor;

  public ShooterSubsystem() {
    shooterMotor = new Spark(0);
  }

  @Override
  public void close() {
    shooterMotor.close();
  }

  public Command run() {
    return runShooter(ShooterConstants.kShooterSpeed);
  }

  public Command run(double speed) {
    return runShooter(speed);
  }

  public Command stop() {
    return this.runOnce(this::stopShooter);
  }

  public Command runShooter(DoubleSupplier speed) {
    return this.runEnd(() -> this.setShooterSpeed(speed.getAsDouble()), this::stopShooter);
  }

  public Command runShooter(double speed) {
    return runShooter(() -> speed);
  }

  private void setShooterSpeed(double speed) {
    shooterMotor.setVoltage(speed * 12.0);
  }

  private void stopShooter() {
    shooterMotor.stopMotor();
  }
}
