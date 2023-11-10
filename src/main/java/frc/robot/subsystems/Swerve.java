// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.BackLeftModule;
import frc.robot.Constants.SwerveConstants.BackRightModule;
import frc.robot.Constants.SwerveConstants.FrontLeftModule;
import frc.robot.Constants.SwerveConstants.FrontRightModule;
import frc.robot.util.AllianceUtil;

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

  private PIDController vxController = new PIDController(SwerveConstants.kpTranslation, 0, 0);
  private PIDController vyController = new PIDController(SwerveConstants.kpTranslation, 0, 0);
  private PIDController vthetaController = new PIDController(SwerveConstants.kpRotation, 0, 0);

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
    driveFieldOriented(forward, strafe, turn, false, false);
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
  public void driveFieldOriented(double forward, double strafe, double turn, boolean usePID, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, -strafe, turn, getRotation());
    setChassisSpeeds(chassisSpeeds, usePID, isOpenLoop);
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    driveRobotOriented(forward, strafe, turn, false, false);
  }

  public void driveRobotOriented(double forward, double strafe, double turn, boolean usePID, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forward, -strafe, turn);
    setChassisSpeeds(chassisSpeeds, false, isOpenLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds, false, false);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds, boolean usePID, boolean isOpenLoop) {
    // Open loop compensation to correct for skewing
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    if (SwerveConstants.chassisSkewCorrection) {
      double dt = 0.02;

      Pose2d robotPoseVelocity = new Pose2d(speeds.vxMetersPerSecond * dt,
          speeds.vyMetersPerSecond * dt, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));

      Twist2d twistVelocity = new Pose2d().log(robotPoseVelocity);

      speeds = new ChassisSpeeds(twistVelocity.dx / dt, twistVelocity.dy / dt, twistVelocity.dtheta / dt);
    }

    setModuleStates(swerveKinematics.toSwerveModuleStates(speeds), usePID, isOpenLoop);
  }

  public void lockModules() {
    lockModules(false);
  }

  public void lockModules(boolean isOpenLoop) {
    setModuleStates(new SwerveModuleState[] { new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)) }, false, isOpenLoop);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false, false);
  }

  public void setModuleStates(SwerveModuleState[] states, boolean usePID, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxModuleSpeed);

    if (usePID) {
      ChassisSpeeds currentSpeeds = getChassisSpeeds();
      ChassisSpeeds desiredSpeeds = swerveKinematics.toChassisSpeeds(states);

      if ((Math.abs(desiredSpeeds.vxMetersPerSecond) > 0.5 && Math.abs(desiredSpeeds.vyMetersPerSecond) > 0.5)
          && Math.abs(desiredSpeeds.omegaRadiansPerSecond) > Units.degreesToRadians(3.0)) {
        double vx = vxController.calculate(currentSpeeds.vxMetersPerSecond, desiredSpeeds.vxMetersPerSecond);
        double vy = vyController.calculate(currentSpeeds.vyMetersPerSecond, desiredSpeeds.vyMetersPerSecond);
        double vrotation = vthetaController.calculate(currentSpeeds.omegaRadiansPerSecond,
            desiredSpeeds.omegaRadiansPerSecond);

        desiredSpeeds = new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond + vx, desiredSpeeds.vyMetersPerSecond + vy,
            desiredSpeeds.omegaRadiansPerSecond + vrotation);

        states = swerveKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxModuleSpeed);
      }
    }

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

    SmartDashboard.putNumberArray("Module States", moduleStates);

    double[] desiredStates = new double[] { frontLeftModule.getDesiredState().angle.getDegrees(),
        frontLeftModule.getDesiredState().speedMetersPerSecond,
        frontRightModule.getDesiredState().angle.getDegrees(),
        frontRightModule.getDesiredState().speedMetersPerSecond, backLeftModule.getDesiredState().angle.getDegrees(),
        backLeftModule.getDesiredState().speedMetersPerSecond, backRightModule.getDesiredState().angle.getDegrees(),
        backRightModule.getDesiredState().speedMetersPerSecond };

    SmartDashboard.putNumberArray("Desired States", desiredStates);

    SmartDashboard.putNumber("Robot Angle", Rotation2d.fromDegrees(-navx.getAngle()).getDegrees());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.setState(targetStates[0], isOpenLoop);
    frontRightModule.setState(targetStates[1], isOpenLoop);
    backLeftModule.setState(targetStates[2], isOpenLoop);
    backRightModule.setState(targetStates[3], isOpenLoop);

    swerveOdometry.update(getRotation(), getModulePositions());

    field.setRobotPose(AllianceUtil.convertToBlueOrigin(swerveOdometry.getPoseMeters()));

    frontLeftModule.updateTelemetry("Front Left");
    frontRightModule.updateTelemetry("Front Right");
    backLeftModule.updateTelemetry("Back Left");
    backRightModule.updateTelemetry("Back Right");

    publishModuleStates();
  }
}