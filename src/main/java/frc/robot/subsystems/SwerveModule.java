// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Alert;
import frc.robot.util.LogUtil;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class SwerveModule {
  private String moduleName;
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private CANCoder canCoder;

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

  public SwerveModule(String moduleName, int driveID, int turnID, int encoderID, Rotation2d angleOffset) {
    this.moduleName = moduleName;
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    canCoder = new CANCoder(encoderID);

    this.angleOffset = angleOffset;

    configureDriveMotor();
    configureTurnMotor();
    configureAngleEncoder();

    resetToAbsolute();

    if (RobotBase.isReal()) {
      LogUtil.recordMetadata(moduleName + " Drive Firmware", driveMotor.getFirmwareString());
      LogUtil.recordMetadata(moduleName + " Turn Firmware", turnMotor.getFirmwareString());
      LogUtil.recordMetadata(moduleName + " CANCoder Firmware", canCoder.getFirmwareVersion());
    }
  }

  public SwerveModule(String moduleName, int driveID, int turnID, int encoderID) {
    this(moduleName, driveID, turnID, encoderID, new Rotation2d(0));
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
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

    drivePID = driveMotor.getPIDController();

    drivePID.setP(SwerveConstants.driveP);
    drivePID.setOutputRange(-1, 1);

    driveEncoder.setPositionConversionFactor(SwerveConstants.drivePositionConversion);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveVelocityConversion);
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
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

    turnEncoder.setPositionConversionFactor(SwerveConstants.turnPositionConversion);

    turnPID = turnMotor.getPIDController();

    turnPID.setP(SwerveConstants.turnP);
    turnPID.setD(SwerveConstants.turnD);
    turnPID.setOutputRange(-1.0, 1.0);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(-Math.PI);
    turnPID.setPositionPIDWrappingMaxInput(Math.PI);
  }

  private void configureAngleEncoder() {
    canCoder.configFactoryDefault();

    canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.configSensorDirection(SwerveConstants.canCoderInverted);
  }

  private void resetToAbsolute() {
    Rotation2d position = Rotation2d
        .fromDegrees(canCoder.getAbsolutePosition() - angleOffset.getDegrees());

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

  public void updateTelemetry() {
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

  public Command getPrematchCommand(Consumer<String> onInfoAlert, Consumer<String> onWarningAlert,
      Consumer<String> onErrorAlert) {
    return Commands.sequence(
        // Check for errors in drive motor
        Commands.runOnce(() -> {
          REVLibError error = driveMotor.getLastError();

          if (error != REVLibError.kOk) {
            onErrorAlert.accept(moduleName + " Drive Motor error: " + error.name());
          } else {
            onInfoAlert.accept(moduleName + " Drive Motor contains no errors");
          }
        }),
        // Check for errors in turn motor
        Commands.runOnce(() -> {
          REVLibError error = turnMotor.getLastError();

          if (error != REVLibError.kOk) {
            onErrorAlert.accept(moduleName + " Turn Motor error: " + error.name());
          } else {
            onInfoAlert.accept(moduleName + " Turn Motor contains no errors");
          }
        }),
        // Check for errors in CANCoder
        Commands.runOnce(() -> {
          CANCoderFaults faults = new CANCoderFaults();
          canCoder.getFaults(faults);

          boolean errorDetected = false;

          if (faults.hasAnyFault()) {
            if (faults.APIError) {
              onErrorAlert.accept(moduleName + " CANCoder API fault detected");
            }
            if (faults.HardwareFault) {
              onErrorAlert.accept(moduleName + " CANCoder hardware fault detected");
            }
            if (faults.ResetDuringEn) {
              onErrorAlert.accept(moduleName + " CANCoder was reset or booted up while robot was enabled");
            }
            if (faults.UnderVoltage) {
              onErrorAlert.accept(moduleName + " CANCoder is receiving less than 6.5V");
            }
            if (faults.MagnetTooWeak) {
              onErrorAlert.accept(moduleName + " CANCoder magnet is too weak");
            }
            errorDetected = true;
          }

          ErrorCode lastError = canCoder.getLastError();
          if (lastError != ErrorCode.OK) {
            onErrorAlert.accept(moduleName + " CANCoder error: " + lastError.name());
            errorDetected = true;
          }

          if (!errorDetected) {
            onInfoAlert.accept(moduleName + " CANCoder has no errors");
          }
        }),
        // Check if drive motor is in brake mode
        Commands.runOnce(() -> {
          if (driveMotor.getIdleMode() != IdleMode.kBrake) {
            onWarningAlert
                .accept(moduleName + " Drive Motor (Motor ID: " + driveMotor.getDeviceId() + ") is not in brake mode");
          } else {
            onInfoAlert
                .accept(moduleName + " Drive Motor (Motor ID: " + driveMotor.getDeviceId() + ") is in brake mode");
          }
        }),
        // Check if turn motor is in coast mode
        Commands.runOnce(() -> {
          if (turnMotor.getIdleMode() != IdleMode.kCoast) {
            onWarningAlert
                .accept(moduleName + " Turn Motor (Motor ID: " + turnMotor.getDeviceId() + ") is not in coast mode");
          } else {
            onInfoAlert
                .accept(moduleName + " Turn Motor (Motor ID: " + turnMotor.getDeviceId() + ") is in coast mode");
          }
        }));
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
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
  }
}