// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystem;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controllers.VirtualXboxController;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public abstract class VirtualSubsystem extends SubsystemBase {
  protected List<Alert> systemAlerts = new ArrayList<Alert>();
  protected String systemStatus = "Pre-Match not ran";

  public final void clearAlerts() {
    for (Alert alert : systemAlerts) {
      alert.removeFromGroup();
    }

    systemAlerts.clear();
  }

  public final void cancelCurrentCommand() {
    Command currentCommand = getCurrentCommand();
    Command defaultCommand = getDefaultCommand();

    if (currentCommand != null && !(defaultCommand != null && currentCommand == defaultCommand)) {
      currentCommand.cancel();
    }
  }

  private final void addAlert(Alert alert) {
    alert.set(true);
    systemAlerts.add(alert);
  }

  public final void addInfo(String message) {
    addAlert(new Alert(getName() + "/Alerts", message, AlertType.INFO));
  }

  public final void addWarning(String message) {
    addAlert(new Alert(getName() + "/Alerts", message, AlertType.WARNING));
  }

  public final void addError(String message) {
    addAlert(new Alert(getName() + "/Alerts", message, AlertType.ERROR));
    setSystemStatus("Prematch failed with reason: \"" + message + "\"");
  }

  public final void setSystemStatus(String status) {
    systemStatus = status;
  }

  public final String getSystemStatus() {
    return systemStatus;
  }

  public final boolean containsErrors() {
    for (Alert alert : systemAlerts) {
      if (alert.getType() == AlertType.ERROR) {
        return true;
      }
    }

    return false;
  }

  public CommandBase getPrematchCheckCommand(VirtualXboxController controller) {
    return Commands.none();
  }
}
