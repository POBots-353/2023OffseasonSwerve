// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.BackLeftModule;
import frc.robot.Constants.SwerveConstants.BackRightModule;
import frc.robot.Constants.SwerveConstants.FrontLeftModule;
import frc.robot.Constants.SwerveConstants.FrontRightModule;
import frc.robot.util.AllianceUtil;

public class Swerve extends VirtualSubsystem {
  private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(SwerveConstants.wheelLocations);

  private SwerveModule frontLeftModule = new SwerveModule("Front Left", FrontLeftModule.driveID, FrontLeftModule.turnID,
      FrontLeftModule.encoderID, FrontLeftModule.angleOffset);

  private SwerveModule frontRightModule = new SwerveModule("Front Right", FrontRightModule.driveID,
      FrontRightModule.turnID, FrontRightModule.encoderID, FrontRightModule.angleOffset);

  private SwerveModule backLeftModule = new SwerveModule("Back Left", BackLeftModule.driveID, BackLeftModule.turnID,
      BackLeftModule.encoderID, BackLeftModule.angleOffset);

  private SwerveModule backRightModule = new SwerveModule("Back Right", BackRightModule.driveID, BackRightModule.turnID,
      BackRightModule.encoderID, BackRightModule.angleOffset);

  private AHRS navx = new AHRS(SPI.Port.kMXP, (byte) SwerveConstants.odometryUpdateFrequency);

  private SwerveDriveOdometry swerveOdometry;

  private Field2d field = new Field2d();

  private final double prematchDriveDelay = 1.0;
  private final double prematchTranslationalTolerance = 0.1;

  /** Creates a new Swerve. */
  public Swerve() {
    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, navx.getRotation2d(), getModulePositions());

    SmartDashboard.putData("Swerve/Field", field);

    // Puts the Gyro on network tables
    SmartDashboard.putData("Swerve/Gyro", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> getRotation().getDegrees(), null);
      }
    });

    // Puts the navx's accelerometer on network tables
    SmartDashboard.putData("Swerve/NavX Accelerometer", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("3AxisAccelerometer");
        builder.addDoubleProperty("X", navx::getWorldLinearAccelX, null);
        builder.addDoubleProperty("Y", navx::getWorldLinearAccelY, null);
        builder.addDoubleProperty("Z", navx::getWorldLinearAccelZ, null);
      }
    });

    // (idk if we'll need the built in accelerometer but you never know)
    SmartDashboard.putData("Swerve/Built-in Accelerometer", new BuiltInAccelerometer());

    // Puts the swerve drive widget on network tables
    SmartDashboard.putData("Swerve/Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getAngle().getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getAngle().getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getAngle().getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getAngle().getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getRotation().getDegrees(), null);
      }
    });

    DataLogManager.log("NavX Firmware: " + navx.getFirmwareVersion());

    Timer.delay(1.00);

    frontLeftModule.resetToAbsolute();
    frontRightModule.resetToAbsolute();
    backLeftModule.resetToAbsolute();
    backRightModule.resetToAbsolute();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { frontLeftModule.getModulePosition(), frontRightModule.getModulePosition(),
        backLeftModule.getModulePosition(), backRightModule.getModulePosition() };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { frontLeftModule.getModuleState(), frontRightModule.getModuleState(),
        backLeftModule.getModuleState(), backRightModule.getModuleState() };
  }

  public ChassisSpeeds getFudgeFactoredSpeeds(ChassisSpeeds speeds) {
    var factored = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(new Rotation2d(speeds.omegaRadiansPerSecond * SwerveConstants.chassisSkewFudgeFactor).unaryMinus());

    return new ChassisSpeeds(factored.getX(), factored.getY(), speeds.omegaRadiansPerSecond);
  }

  /**
   * Drives the robot relative to the field
   * 
   * @param forward The forward velocity of the robot. Positive is going away from
   *                your alliance wall
   * @param strafe  The sideways velocity of the robot. Positive is going to the
   *                right when you are standing behind the alliance wall
   * @param turn    The angular velocity of the robot (CCW is +)
   */
  public void driveFieldOriented(double forward, double strafe, double turn) {
    driveFieldOriented(forward, strafe, turn, true, false, false);
  }

  /**
   * Drives the robot relative to the field
   * 
   * @param forward     The forward velocity of the robot. Positive is going away
   *                    from your alliance wall
   * @param strafe      The sideways velocity of the robot. Positive is going to
   *                    the right when you are standing behind the alliance wall
   * @param turn        The angular velocity of the robot (CCW is +)
   * @param fudgeFactor Weather or not to adjust the translation of the robot
   *                    relative to the turning speed
   * @param isOpenLoop  Weather the drive motors should be open loop
   */
  public void driveFieldOriented(double forward, double strafe, double turn, boolean fudgeFactor, boolean isOpenLoop,
      boolean allowTurnInPlace) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, -strafe, turn, getRotation());
    setChassisSpeeds(chassisSpeeds, fudgeFactor, isOpenLoop, allowTurnInPlace);
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    driveRobotOriented(forward, strafe, turn, true, false, false);
  }

  public void driveRobotOriented(double forward, double strafe, double turn, boolean fudgeFactor, boolean isOpenLoop,
      boolean allowTurnInPlace) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, -strafe, turn);
    setChassisSpeeds(chassisSpeeds, fudgeFactor, isOpenLoop, allowTurnInPlace);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds, true, false, false);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean fudgeFactor, boolean isOpenLoop,
      boolean allowTurnInPlace) {
    // Open loop compensation to correct for skewing
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    double dt = 0.02;

    Pose2d robotPoseVelocity = new Pose2d(speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

    Twist2d twistVelocity = new Pose2d().log(robotPoseVelocity);

    speeds = new ChassisSpeeds(twistVelocity.dx / dt, twistVelocity.dy / dt, twistVelocity.dtheta / dt);

    if (fudgeFactor) {
      speeds = getFudgeFactoredSpeeds(speeds);
    }

    setModuleStates(swerveKinematics.toSwerveModuleStates(speeds), isOpenLoop, allowTurnInPlace);
  }

  public void lockModules() {
    setModuleStates(new SwerveModuleState[] { new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)) }, false, true);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false, false);
  }

  public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop, boolean allowTurnInPlace) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxModuleSpeed);

    frontLeftModule.setState(states[0], isOpenLoop, allowTurnInPlace);
    frontRightModule.setState(states[1], isOpenLoop, allowTurnInPlace);
    backLeftModule.setState(states[2], isOpenLoop, allowTurnInPlace);
    backRightModule.setState(states[3], isOpenLoop, allowTurnInPlace);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  public void zeroYaw() {
    Pose2d originalOdometryPosition = getPose();

    navx.zeroYaw();

    swerveOdometry.resetPosition(new Rotation2d(0.0), getModulePositions(),
        new Pose2d(originalOdometryPosition.getTranslation(), new Rotation2d(0.0)));
  }

  public Rotation2d getRotation() {
    return navx.getRotation2d();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose, Trajectory trajectory) {
    resetOdometry(pose);

    field.getObject("trajectory").setTrajectory(trajectory);
  }

  public void clearTrajectory() {
    field.getObject("trajectory").setPoses(new Pose2d[] {});
  }

  private void publishModuleStates() {
    double[] moduleStates = new double[] { frontLeftModule.getAngle().getDegrees(), frontLeftModule.getVelocity(),
        frontRightModule.getAngle().getDegrees(), frontRightModule.getVelocity(),
        backLeftModule.getAngle().getDegrees(), backLeftModule.getVelocity(), backRightModule.getAngle().getDegrees(),
        backRightModule.getVelocity() };

    SmartDashboard.putNumberArray("Swerve/Module States", moduleStates);

    double[] desiredStates = new double[] { frontLeftModule.getState().angle.getDegrees(),
        frontLeftModule.getState().speedMetersPerSecond, frontRightModule.getState().angle.getDegrees(),
        frontRightModule.getState().speedMetersPerSecond, backLeftModule.getState().angle.getDegrees(),
        backLeftModule.getState().speedMetersPerSecond, backRightModule.getState().angle.getDegrees(),
        backRightModule.getState().speedMetersPerSecond };

    SmartDashboard.putNumberArray("Swerve/Desired States", desiredStates);

    SmartDashboard.putNumber("Swerve/Robot Angle", Rotation2d.fromDegrees(-navx.getAngle()).getDegrees());
  }

  public void updateOdometry() {
    swerveOdometry.update(getRotation(), getModulePositions());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.periodic();
    frontRightModule.periodic();
    backLeftModule.periodic();
    backRightModule.periodic();

    field.setRobotPose(AllianceUtil.convertToBlueOrigin(swerveOdometry.getPoseMeters()));

    SmartDashboard.putString("Swerve/Status", getSystemStatus());

    publishModuleStates();
  }

  @Override
  public CommandBase getPrematchCheckCommand(VirtualXboxController controller) {
    return Commands.sequence(
        // Make sure gyro is connected
        Commands.runOnce(() -> {
          if (!navx.isConnected()) {
            addError("NavX is not connected");
          } else {
            addInfo("NavX is connected");
          }
        }),
        // Test gyro zeroing
        Commands.runOnce(() -> {
          controller.setStartButton(true);
          controller.setBackButton(true);
        }),
        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> {
          if (Math.abs(getRotation().getDegrees()) > 0.05) {
            addError("Gyro failed to zero");
          } else {
            addInfo("Gyro zero successful");
          }

          controller.clearVirtualButtons();
        }),
        // Test all modules
        Commands.sequence(
            frontLeftModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError),
            frontRightModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError),
            backLeftModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError),
            backRightModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError)),
        // Test forward speed
        Commands.runOnce(() -> {
          controller.setLeftY(-1.0);
        }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (getChassisSpeeds().vxMetersPerSecond < Units.feetToMeters(10.0)) {
            addError("Forward speed too slow");
          } else if (Math.abs(getChassisSpeeds().vyMetersPerSecond) > prematchTranslationalTolerance) {
            addError("Strafe speed too high");
          } else {
            addInfo("Forward drive successful");
          }

          controller.clearVirtualAxes();
        }),
        // Test slowing down to 0 m/s
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (Math.abs(getChassisSpeeds().vxMetersPerSecond) > Units.feetToMeters(0.1)
              || Math.abs(getChassisSpeeds().vyMetersPerSecond) > Units.feetToMeters(0.1)) {
            addError("Robot moving too fast");
          } else {
            addInfo("Slow down successful");
          }

          controller.clearVirtualAxes();
        }),
        // Test backward speed
        Commands.runOnce(() -> {
          controller.setLeftY(1.0);
        }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (getChassisSpeeds().vxMetersPerSecond > Units.feetToMeters(-10.0)) {
            addError("Backward speed too slow");
          } else if (Math.abs(getChassisSpeeds().vyMetersPerSecond) > prematchTranslationalTolerance) {
            addError("Strafe speed too high");
          } else {
            addInfo("Backward drive successful");
          }

          controller.clearVirtualAxes();
        }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test left speed
        Commands.runOnce(() -> {
          controller.setLeftX(-1.0);
        }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (getChassisSpeeds().vyMetersPerSecond < Units.feetToMeters(10.0)) {
            addError("Left speed too slow");
          } else if (Math.abs(getChassisSpeeds().vxMetersPerSecond) > prematchTranslationalTolerance) {
            addError("Forward/Backward speed too high");
          } else {
            addInfo("Left drive sucessful");
          }

          controller.clearVirtualAxes();
        }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test right speed
        Commands.runOnce(() -> {
          controller.setLeftX(1.0);
        }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (getChassisSpeeds().vyMetersPerSecond > Units.feetToMeters(-10.0)) {
            addError("Right speed too slow");
          } else if (Math.abs(getChassisSpeeds().vxMetersPerSecond) > prematchTranslationalTolerance) {
            addError("Forward/Backward speed too high");
          } else {
            addInfo("Right drive successful");
          }

          controller.clearVirtualAxes();
        }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test angular CW speed
        Commands.runOnce(() -> {
          controller.setRightX(1.0);
        }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (getChassisSpeeds().omegaRadiansPerSecond > Units.degreesToRadians(-160.0)) {
            addError("Clockwise rotation too slow");
          } else {
            addInfo("Clockwise rotation successful");
          }

          controller.clearVirtualAxes();
        }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test angular CCW speed
        Commands.runOnce(() -> {
          controller.setRightX(-1.0);
        }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(() -> {
          if (getChassisSpeeds().omegaRadiansPerSecond < Units.degreesToRadians(160.0)) {
            addError("Counter Clockwise rotation too slow");
          } else {
            addInfo("Counter Clockwise rotation successful");
          }

          controller.clearVirtualAxes();
        }));
  }
}