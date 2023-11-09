// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.BackLeftModule;
import frc.robot.Constants.SwerveConstants.BackRightModule;
import frc.robot.Constants.SwerveConstants.FrontLeftModule;
import frc.robot.Constants.SwerveConstants.FrontRightModule;

public class Swerve extends SubsystemBase {
  private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(SwerveConstants.wheelLocations);

  private SwerveModule frontLeftModule = new SwerveModule(FrontLeftModule.driveID, FrontLeftModule.turnID,
      FrontLeftModule.encoderID, FrontLeftModule.angleOffset);

  private SwerveModule frontRightModule = new SwerveModule(FrontRightModule.driveID, FrontRightModule.turnID,
      FrontRightModule.encoderID, FrontRightModule.angleOffset);

  private SwerveModule backLeftModule = new SwerveModule(BackLeftModule.driveID, BackLeftModule.turnID,
      BackLeftModule.encoderID, BackLeftModule.angleOffset);

  private SwerveModule backRightModule = new SwerveModule(BackRightModule.driveID, BackRightModule.turnID,
      BackRightModule.encoderID, BackRightModule.angleOffset);

  private SwerveModuleState[] targetStates = { new SwerveModuleState(), new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState() };

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private SwerveDriveOdometry swerveOdometry;

  private Field2d field = new Field2d();

  private boolean isOpenLoop = false;

  /** Creates a new Swerve. */
  public Swerve() {
    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, navx.getRotation2d(), getModulePositions());

    SmartDashboard.putData("Field", field);

    // Puts the Gyro on the dashboard
    SmartDashboard.putData("Gyro", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> getRotation().getDegrees(), null);
      }
    });

    // Puts the swerve drive widget on the dashboard
    SmartDashboard.putData("Swerve Drive", new Sendable() {
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
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { frontLeftModule.getModulePosition(), frontRightModule.getModulePosition(),
        backLeftModule.getModulePosition(), backRightModule.getModulePosition() };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { frontLeftModule.getModuleState(), frontRightModule.getModuleState(),
        backLeftModule.getModuleState(), backRightModule.getModuleState() };
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
    driveFieldOriented(forward, strafe, turn, false);
  }

  /**
   * Drives the robot relative to the field
   * 
   * @param forward    The forward velocity of the robot. Positive is going away
   *                   from your alliance wall
   * @param strafe     The sideways velocity of the robot. Positive is going to
   *                   the right when you are standing behind the alliance wall
   * @param turn       The angular velocity of the robot (CCW is +)
   * @param isOpenLoop Weather the drive motors should be open loop
   */
  public void driveFieldOriented(double forward, double strafe, double turn, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, -strafe, turn, getRotation());
    setChassisSpeeds(chassisSpeeds, isOpenLoop);
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    driveRobotOriented(forward, strafe, turn, false);
  }

  public void driveRobotOriented(double forward, double strafe, double turn, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, -strafe, turn);
    setChassisSpeeds(chassisSpeeds, isOpenLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds, false);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    // Open loop compensation to correct for skewing
    // Made by Team 254
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    if (SwerveConstants.chassisSkewCorrection) {
      double dt = 0.02;

      Pose2d robotPoseVelocity = new Pose2d(speeds.vxMetersPerSecond * dt,
          speeds.vyMetersPerSecond * dt, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

      Twist2d twistVelocity = new Pose2d().log(robotPoseVelocity);

      speeds = new ChassisSpeeds(twistVelocity.dx / dt, twistVelocity.dy / dt, twistVelocity.dtheta / dt);
    }

    setModuleStates(swerveKinematics.toSwerveModuleStates(speeds), isOpenLoop);
  }

  public void lockModules() {
    lockModules(false);
  }

  public void lockModules(boolean isOpenLoop) {
    setModuleStates(new SwerveModuleState[] { new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)) }, isOpenLoop);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false);
  }

  public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxModuleSpeed);

    targetStates = states;
    this.isOpenLoop = isOpenLoop;
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  public void zeroYaw() {
    Pose2d originalOdometryPosition = getPose();

    navx.zeroYaw();

    swerveOdometry.resetPosition(getRotation(), getModulePositions(), originalOdometryPosition);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.setState(targetStates[0], isOpenLoop);
    frontRightModule.setState(targetStates[1], isOpenLoop);
    backLeftModule.setState(targetStates[2], isOpenLoop);
    backRightModule.setState(targetStates[3], isOpenLoop);

    field.setRobotPose(swerveOdometry.update(getRotation(), getModulePositions()));

    SmartDashboard.putNumber("Front Left Velocity", frontLeftModule.getVelocity());
    SmartDashboard.putNumber("Front Left Rotation", frontLeftModule.getAngle().getDegrees());
    SmartDashboard.putNumber("Front Left Absolute Rotation", frontLeftModule.getAbsoluteAngle().getDegrees());

    SmartDashboard.putNumber("Front Right Velocity", frontRightModule.getVelocity());
    SmartDashboard.putNumber("Front Right Rotation", frontRightModule.getAngle().getDegrees());
    SmartDashboard.putNumber("Front Right Absolute Rotation", frontRightModule.getAbsoluteAngle().getDegrees());

    SmartDashboard.putNumber("Back Left Velocity", backLeftModule.getVelocity());
    SmartDashboard.putNumber("Back Left Rotation", backLeftModule.getAngle().getDegrees());
    SmartDashboard.putNumber("Back Left Absolute Rotation", backLeftModule.getAbsoluteAngle().getDegrees());

    SmartDashboard.putNumber("Back Right Velocity", backRightModule.getVelocity());
    SmartDashboard.putNumber("Back Right Rotation", backRightModule.getAngle().getDegrees());
    SmartDashboard.putNumber("Back Right Absolute Rotation", backRightModule.getAbsoluteAngle().getDegrees());
  }
}