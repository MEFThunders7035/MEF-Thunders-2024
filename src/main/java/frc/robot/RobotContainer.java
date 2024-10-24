package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.BasicIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootToAmpCommand;
import frc.robot.commands.ShootToSpeakerCommand;
import frc.robot.commands.SmartIntakeCommand;
import frc.robot.commands.led_commands.LEDIdleCommand;
import frc.robot.simulationSystems.PhotonSim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.PhotonCameraSystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  private final CommandXboxController commandController = new CommandXboxController(0);
  private final XboxController controller = commandController.getHID();

  private final SendableChooser<Command> autoChooser;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem ledSubsystem = LEDSystem.getInstance();

  public RobotContainer() {
    setupNamedCommands();
    loggingInit();
    configureJoystickBindings();
    setDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption(
        "Shoot To Shooter",
        new ShootToSpeakerCommand(shooterSubsystem, intakeSubsystem, armSubsystem, driveSubsystem));
    PhotonCameraSystem.getAprilTagWithID(0); // Load the class before enable.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    if (RobotBase.isSimulation()) {
      simInit();
    }
    setupCamera();
  }

  private void setupCamera() {
    CameraServer.startAutomaticCapture();
  }

  private void loggingInit() {
    DataLogManager.start();
    URCL.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  private Thread simThread;

  public void simInit() {
    simThread =
        new Thread(
            () -> {
              System.out.println("Starting PhotonSim");
              while (true) {
                PhotonSim.update(driveSubsystem.getPose());
                // I do not want to use a busy loop, so I added a delay.
                Timer.delay(0.05);
              }
            },
            "simThread");
    simThread.setDaemon(true);
    simThread.start();
  }

  public void simPeriodic() {
    if (!simThread.isAlive()) {
      simInit(); // If the thread dies, restart it.
      // This is here because sometimes the thread throws an exception and dies.
    }
    // add any simulation specific code here.
    // was made for photonSim, but it's not used.
  }

  private void setupNamedCommands() {
    NamedCommands.registerCommand("Intake", new BasicIntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand(
        "Shoot To Speaker",
        new ShootToSpeakerCommand(shooterSubsystem, intakeSubsystem, armSubsystem, driveSubsystem));
    NamedCommands.registerCommand(
        "Shoot To Amp", new ShootToAmpCommand(shooterSubsystem, intakeSubsystem, armSubsystem));
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveCommands.driveWithController(driveSubsystem, controller));

    // move arm with midi's potentiometer
    armSubsystem.setDefaultCommand(armSubsystem.set(ArmState.IDLE));

    intakeSubsystem.setDefaultCommand(intakeSubsystem.stop());

    shooterSubsystem.setDefaultCommand(shooterSubsystem.stop());

    ledSubsystem.setDefaultCommand(
        new LEDIdleCommand(ledSubsystem, intakeSubsystem).ignoringDisable(true));
  }

  private void configureJoystickBindings() {
    commandController.a().whileTrue(new RunCommand(driveSubsystem::setX, driveSubsystem));

    commandController.b().whileTrue(new SmartIntakeCommand(intakeSubsystem, controller));

    commandController
        .y()
        .whileTrue(
            new ShootToSpeakerCommand(
                shooterSubsystem, intakeSubsystem, armSubsystem, driveSubsystem, controller));

    commandController
        .x()
        .whileTrue(new ShootToAmpCommand(shooterSubsystem, intakeSubsystem, armSubsystem));

    commandController.start().onTrue(driveSubsystem.resetFieldOrientation());

    // This command is here incase the intake gets stuck.
    commandController.back().whileTrue(intakeSubsystem.run(-IntakeConstants.kIntakeSpeed));

    commandController.rightBumper().whileTrue(shooterSubsystem.run(-1));

    // lower level shoot command incase the other one has some issues it will not rotate the robot
    // to face the shooter. which can sometimes break, currently investigating.
    commandController
        .leftBumper()
        .whileTrue(
            new ShootToSpeakerCommand(
                shooterSubsystem, intakeSubsystem, armSubsystem, driveSubsystem));

    commandController.pov(0).whileTrue(armSubsystem.setArmToAmp());
    commandController.pov(180).whileTrue(armSubsystem.setArmToBottom());
  }

  public Command getAutonomousCommand() {
    return autoChooser
        .getSelected()
        .andThen(driveSubsystem.stop())
        .beforeStarting(armSubsystem.runOnce(armSubsystem::resetEncoder));
  }
}
