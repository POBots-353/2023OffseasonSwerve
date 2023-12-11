// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.net.InetAddress;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RadioPing extends CommandBase {
  private Notifier notifier;

  private InetAddress ipAddress;

  private boolean receivedConnection = false;
  private int responseCount = 0;

  /** Creates a new RadioPing. */
  public RadioPing() {
    try {
      ipAddress = InetAddress.getByName("10.3.53.1");
    } catch (Exception e) {
      e.printStackTrace();
    }

    notifier = new Notifier(this::pingIPAddress);
  }

  private void pingIPAddress() {
    try {
      if (ipAddress.isReachable(500)) {
        synchronized (this) {
          receivedConnection = true;
          responseCount++;
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notifier.startPeriodic(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    notifier.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return receivedConnection && responseCount >= 3;
  }
}
