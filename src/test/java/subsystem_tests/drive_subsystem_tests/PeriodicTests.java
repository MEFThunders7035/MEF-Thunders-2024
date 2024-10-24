package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PeriodicTests extends DriveSubsystemTestBase {
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
  void testPeriodic() {
    driveSubsystem.resetOdometry(new Pose2d(3, 2, Rotation2d.fromDegrees(30)));

    driveSubsystem.periodic();
    Field2d field = (Field2d) SmartDashboard.getData("Field");

    assertEquals(Rotation2d.fromDegrees(30), field.getRobotPose().getRotation());
    assertEquals(3, field.getRobotPose().getTranslation().getX());
    assertEquals(2, field.getRobotPose().getTranslation().getY());
  }

  @Test
  void testPeriodicSwerveStateSending() {
    var entry =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Swerve", SwerveModuleState.struct)
            .subscribe(new SwerveModuleState[4]);

    ChassisSpeeds speedsToGo = new ChassisSpeeds(1, 0, 0);
    driveSubsystem.driveRobotRelative(speedsToGo);

    driveSubsystem.periodic();

    NetworkTableInstance.getDefault().waitForListenerQueue(-1);

    var states = entry.get();

    assertNotNull(states[0]);
    assertNotNull(states[1]);
    assertNotNull(states[2]);
    assertNotNull(states[3]);
    var receivedSpeeds =
        SwerveModuleConstants.kDriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {states[0], states[1], states[2], states[3]});

    // Until 2025, ChassisSpeeds equals check is not implemented we'll compare each
    // value separately
    // See: https://github.com/wpilibsuite/allwpilib/pull/6414
    // do this in 2025: {assertEquals(speedsToGo, receivedSpeeds);}
    assertEquals(speedsToGo.vxMetersPerSecond, receivedSpeeds.vxMetersPerSecond, 0.01);
    assertEquals(speedsToGo.vyMetersPerSecond, receivedSpeeds.vyMetersPerSecond, 0.01);
    assertEquals(speedsToGo.omegaRadiansPerSecond, receivedSpeeds.omegaRadiansPerSecond, 0.01);
  }
}
