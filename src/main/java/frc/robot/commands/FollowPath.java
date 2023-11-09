// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.SwerveConstants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class FollowPath extends PPSwerveControllerCommand {
  private Swerve swerve;
  private PathPlannerTrajectory trajectory;

  /** Creates a new FollowPath. */
  public FollowPath(PathPlannerTrajectory trajectory, Swerve swerve) {
    super(trajectory, swerve::getPose,
        new PIDController(AutoConstants.translationalP, AutoConstants.translationalI, AutoConstants.translationalD),
        new PIDController(AutoConstants.translationalP, AutoConstants.translationalI, AutoConstants.translationalD),
        new PIDController(AutoConstants.rotationalP, AutoConstants.rotationalI, AutoConstants.rotationalD),
        swerve::setChassisSpeeds, true, swerve);

    this.swerve = swerve;
    this.trajectory = trajectory;
  }

  public FollowPath(String path, Swerve swerve) {
    this(PathPlanner.loadPath(path, new PathConstraints(AutoConstants.maxVelocity, AutoConstants.maxAcceleration)),
        swerve);
  }

  @Override
  public void initialize() {
    swerve.resetOdometry(trajectory.getInitialHolonomicPose());

    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerve.lockModules();
  }
}