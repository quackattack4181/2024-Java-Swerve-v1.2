package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final Rotation2d angleOffset;
  public final int canCoderID;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param angleOffset 
   * @param canCoderID
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, Rotation2d angleOffset, int canCoderID) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.angleOffset = angleOffset;
    this.canCoderID = canCoderID;
  }
}
