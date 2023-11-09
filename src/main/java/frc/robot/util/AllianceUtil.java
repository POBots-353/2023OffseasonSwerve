package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AllianceUtil {
  public static final Pose2d convertToBlueOrigin(Pose2d pose) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return pose.relativeTo(FieldConstants.redAllianceOrigin);
    }

    return pose;
  }
}
