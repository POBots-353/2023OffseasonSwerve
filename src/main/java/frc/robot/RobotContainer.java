// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.controllers.VirtualXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.FollowAprilTag;
import frc.robot.commands.FollowPath;
import frc.robot.commands.StartupConnectionCheck;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Alert;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;
import frc.robot.util.Alert.AlertType;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Swerve swerve = new Swerve();

  private PowerDistribution powerDistribution = new PowerDistribution();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private PersistentSendableChooser<String> batteryChooser = new PersistentSendableChooser<>("Battery Number");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final VirtualXboxController driverController = new VirtualXboxController(
      OperatorConstants.driverControllerPort);

  private List<Alert> robotAlerts = new ArrayList<Alert>();

  private Alert generalPrematchAlert = new Alert("", AlertType.INFO);
  private Alert swervePrematchAlert = new Alert("", AlertType.INFO);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoChooser();
    configureBatteryChooser();
    configurePreMatchChecks();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Power Distribution Panel", powerDistribution);

    LogUtil.recordMetadata("Battery Number", batteryChooser.getSelectedName());
    LogUtil.recordMetadata("Battery Nickname", batteryChooser.getSelected());

    swerve.setDefaultCommand(
        new SwerveDrive(driverController::getLeftY, driverController::getLeftX,
            driverController::getRightX, driverController::getRightY, driverController::getLeftBumper,
            SwerveConstants.maxTranslationalSpeed, SwerveConstants.maxAngularSpeed, swerve));

    Command onConnectionSuccessful = Commands.runOnce(() -> {
      addInfoAlert("Connection Successful!");
      DataLogManager.log("Connection successful!");
    }).ignoringDisable(true);

    Command onConnectionFailed = Commands.runOnce(() -> {
      addErrorAlert("Connection Failed!");
      DataLogManager.log("Connection Failed!");
    }).ignoringDisable(true);

    new StartupConnectionCheck(onConnectionSuccessful, onConnectionFailed).schedule();
  }

  public void updateSwerveOdometry() {
    swerve.updateOdometry();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.x().whileTrue(swerve.run(swerve::lockModules).withName("Lock Modules"));

    driverController.start().and(driverController.back())
        .toggleOnTrue(Commands.runOnce(swerve::zeroYaw).ignoringDisable(true));
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("[Demo] Follow April Tag", new FollowAprilTag(swerve));
    autoChooser.addOption("[PathPlanner] New Path (Test Path 1)",
        new FollowPath("New Path", swerve).andThen(swerve.run(swerve::lockModules)));
    autoChooser.addOption("[PathPlanner] New New Path (Test Path 2)",
        new FollowPath("New New Path", swerve).andThen(swerve.run(swerve::lockModules)));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBatteryChooser() {
    batteryChooser.addOption("2019.5 #2", "Al");
    batteryChooser.addOption("2019.5 #3", "Daniel");
    batteryChooser.addOption("2020 #1", "2020 #1");
    batteryChooser.addOption("2020 #2", "2020 #2");
    batteryChooser.addOption("2021 #1", "Fred");
    batteryChooser.addOption("2024 #1", "2024 #1");
    batteryChooser.addOption("2024 #2", "2024 #2");

    SmartDashboard.putData("Battery Chooser", batteryChooser);
  }

  private void configurePreMatchChecks() {
    CommandBase generalPreMatch = Commands.sequence(
        Commands.runOnce(() -> clearRobotAlerts()),
        Commands.runOnce(() -> {
          if (!driverController.getHID().isConnected()) {
            addErrorAlert("Driver controller is not connected");
          } else {
            addInfoAlert("Driver controller is connected");
          }
        }),
        Commands.runOnce(() -> {
          if (!DriverStation.getJoystickIsXbox(OperatorConstants.driverControllerPort)) {
            addErrorAlert("Controller port 0 is not an Xbox controller");
          } else {
            addInfoAlert("Controller port 0 is the correct joystick type (Xbox Controller)");
          }
        }))
        .until(this::errorsPresent)
        .andThen(() -> {
          removeAlert(generalPrematchAlert);
          if (errorsPresent()) {
            generalPrematchAlert = new Alert("General Pre-Match Failed!", AlertType.ERROR);
          } else {
            generalPrematchAlert = new Alert("General Pre-Match Successful!", AlertType.INFO);
          }
          addAlert(generalPrematchAlert);
        })
        .unless(DriverStation::isFMSAttached)
        .withName("General Pre-Match");

    CommandBase swervePreMatch = swerve.buildPrematch(driverController)
        .finallyDo((interrupted) -> {
          removeAlert(swervePrematchAlert);
          if (swerve.containsErrors()) {
            swervePrematchAlert = new Alert("Swerve Pre-Match Failed!", AlertType.ERROR);
          } else {
            swervePrematchAlert = new Alert("Swerve Pre-Match Successful!", AlertType.INFO);
          }
          addAlert(swervePrematchAlert);
        })
        .unless(DriverStation::isFMSAttached)
        .withName("Swerve Pre-Match");

    SmartDashboard.putData("Full Pre-Match", Commands.sequence(
        Commands.runOnce(() -> clearRobotAlerts()),
        generalPreMatch.asProxy(),
        swervePreMatch.asProxy(),
        Commands.runOnce(() -> {
          if (!errorsPresent()) {
            addInfoAlert("Pre-Match Successful! Good luck in the next match! Let's kick bot!");
          } else {
            addErrorAlert("Pre-Match Failed!");
          }
        }))
        .unless(DriverStation::isFMSAttached)
        .withName("Full Pre-Match"));

    SmartDashboard.putData("General Pre-Match Check", generalPreMatch.asProxy());
    SmartDashboard.putData("Swerve/Swerve Pre-Match Check", swervePreMatch.asProxy());
  }

  private void clearRobotAlerts() {
    for (Alert alert : robotAlerts) {
      alert.removeFromGroup();
    }

    robotAlerts.clear();
  }

  private void removeAlert(Alert alert) {
    alert.removeFromGroup();
    robotAlerts.remove(alert);
  }

  private boolean errorsPresent() {
    for (Alert alert : robotAlerts) {
      if (alert.getType() == AlertType.ERROR) {
        return true;
      }
    }

    return false;
  }

  private void addAlert(Alert alert) {
    alert.set(true);
    robotAlerts.add(alert);
  }

  public void addErrorAlert(String message) {
    addAlert(new Alert(message, AlertType.ERROR));
  }

  public void addWarningAlert(String message) {
    addAlert(new Alert(message, AlertType.WARNING));
  }

  public void addInfoAlert(String message) {
    addAlert(new Alert(message, AlertType.INFO));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
