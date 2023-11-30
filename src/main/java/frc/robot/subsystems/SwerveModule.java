// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private CANCoder canEncoder;

  private Rotation2d angleOffset;

  private SparkMaxPIDController drivePID;
  private SparkMaxPIDController turnPID;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
      SwerveConstants.driveKv, SwerveConstants.driveKa);

  private SwerveModuleState desiredState = new SwerveModuleState();

  private double prevVelocity = 0.0;

  private Alert driveMotorTemperatureAlert = new Alert("Swerve/Alerts", "drive motor temperature is above 50°C",
      AlertType.WARNING);
  private Alert turnMotorTemperatureAlert = new Alert("Swerve/Alerts", "turn motor temperature is above 50°C",
      AlertType.WARNING);

  public SwerveModule(int driveID, int turnID, int encoderID, Rotation2d angleOffset) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    canEncoder = new CANCoder(encoderID);

    this.angleOffset = angleOffset;

    configureDriveMotor();
    configureTurnMotor();
    configureAngleEncoder();

    resetToAbsolute();
  }

  public SwerveModule(int driveID, int turnID, int encoderID) {
    this(driveID, turnID, encoderID, new Rotation2d(0));
  }

  public void configureDriveMotor() {
    driveMotor.restoreFactoryDefaults();

    driveMotor.setInverted(SwerveConstants.driveMotorInverted);

    driveMotor.setOpenLoopRampRate(SwerveConstants.openLoopRamp);
    driveMotor.setClosedLoopRampRate(SwerveConstants.closedLoopRamp);

    driveMotor.enableVoltageCompensation(SwerveConstants.voltageCompensation);
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveCurrentLimit);

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000 / SwerveConstants.odometryUpdateFrequency);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);

    drivePID = driveMotor.getPIDController();

    drivePID.setP(SwerveConstants.driveP);
    drivePID.setOutputRange(-1, 1);

    driveEncoder.setPositionConversionFactor(SwerveConstants.drivePositionConversion);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveVelocityConversion);

    driveMotor.burnFlash();
  }

  public void configureTurnMotor() {
    turnMotor.restoreFactoryDefaults();

    turnMotor.setInverted(SwerveConstants.turnMotorInverted);

    turnMotor.enableVoltageCompensation(SwerveConstants.voltageCompensation);
    turnMotor.setSmartCurrentLimit(SwerveConstants.turnCurrentLimit);

    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000 / SwerveConstants.odometryUpdateFrequency);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);

    turnEncoder.setPositionConversionFactor(SwerveConstants.turnPositionConversion);

    turnPID = turnMotor.getPIDController();

    turnPID.setP(SwerveConstants.turnP);
    turnPID.setD(SwerveConstants.turnD);
    turnPID.setOutputRange(-1.0, 1.0);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(-Math.PI);
    turnPID.setPositionPIDWrappingMaxInput(Math.PI);

    turnMotor.burnFlash();
  }

  private void configureAngleEncoder() {
    canEncoder.configFactoryDefault();

    canEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    canEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    canEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canEncoder.configSensorDirection(SwerveConstants.canCoderInverted);
  }

  private void resetToAbsolute() {
    Rotation2d position = Rotation2d
        .fromDegrees(canEncoder.getAbsolutePosition() - angleOffset.getDegrees());

    turnEncoder.setPosition(position.getRadians());
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    turnPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);

    double currentVelocity = optimizedState.speedMetersPerSecond;

    if (isOpenLoop) {
      driveMotor.set(currentVelocity / SwerveConstants.maxModuleSpeed);
    } else {
      double feedForward = driveFeedforward.calculate(currentVelocity, (currentVelocity - prevVelocity) / 0.020);

      drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0, feedForward,
          ArbFFUnits.kVoltage);
    }

    desiredState = optimizedState;
    prevVelocity = currentVelocity;
  }

  public void updateTelemetry(String moduleName) {
    // Setpoint data
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Velocity", getVelocity());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Absolute Angle", getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Desired Velocity", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Desired Angle", desiredState.angle.getDegrees());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Velocity Error",
        desiredState.speedMetersPerSecond - getVelocity());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Angle Error",
        desiredState.angle.minus(getAngle()).getDegrees());

    // Other information about the motors (output, voltage, etc)
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Drive Temperature", driveMotor.getMotorTemperature());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Turn Temperature", turnMotor.getMotorTemperature());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Drive Applied Output", driveMotor.getAppliedOutput());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Turn Applied Output", turnMotor.getAppliedOutput());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Drive Output Current", driveMotor.getOutputCurrent());
    SmartDashboard.putNumber("Swerve/" + moduleName + "/Turn Output Current", turnMotor.getOutputCurrent());

    // Alerts for high temperature
    double driveMotorTemperature = driveMotor.getMotorTemperature();
    if (driveMotorTemperature >= SwerveConstants.maxDriveTemperature) {
      driveMotorTemperatureAlert
          .setText(moduleName + " drive motor temperature is above 50°C (" + driveMotorTemperature + "°C)");
      driveMotorTemperatureAlert.set(true);
    } else {
      driveMotorTemperatureAlert.set(false);
    }

    double turnMotorTemperature = turnMotor.getMotorTemperature();
    if (turnMotorTemperature >= SwerveConstants.maxTurnTemperature) {
      turnMotorTemperatureAlert
          .setText(moduleName + " turn motor temperature is above 50°C (" + turnMotorTemperature + "°C)");
      turnMotorTemperatureAlert.set(true);
    } else {
      turnMotorTemperatureAlert.set(false);
    }
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(turnEncoder.getPosition());
  }

  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(canEncoder.getAbsolutePosition());
  }
}