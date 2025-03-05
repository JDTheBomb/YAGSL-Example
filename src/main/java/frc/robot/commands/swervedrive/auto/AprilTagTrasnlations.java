// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class AprilTagTrasnlations {
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final SwerveSubsystem swerveSubsystem;

    public AprilTagTrasnlations(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }
    public Command driveToAprilTag(int apriltagnumber, Translation2d offsetFromApriltag, double rotationOffset){
        Pose2d targetAprilTagPose = aprilTagFieldLayout.getTagPose(apriltagnumber).get().toPose2d();
        return swerveSubsystem.driveToPose(
        //targetAprilTagPose.plus(new Transform2d(new Translation2d(2, 0).rotateBy(targetAprilTagPose.getRotation()),new Rotation2d()))
        new Pose2d().transformBy(
            targetAprilTagPose.minus(
            new Pose2d(
                offsetFromApriltag.rotateBy(targetAprilTagPose.getRotation().plus(
                new Rotation2d(Units.degreesToRadians(-180)))
                ),
                new Rotation2d()
            )
            )
        ).plus(new Transform2d(0, 0, new Rotation2d(Units.degreesToRadians(180))))
        );
    };
    public Command driveToAprilTag(int apriltagnumber, double distanceFromAprilTag, double rotationOffset){
      Pose2d targetAprilTagPose = aprilTagFieldLayout.getTagPose(apriltagnumber).get().toPose2d();
      return swerveSubsystem.driveToPose(
        //targetAprilTagPose.plus(new Transform2d(new Translation2d(2, 0).rotateBy(targetAprilTagPose.getRotation()),new Rotation2d()))
        new Pose2d().transformBy(
          targetAprilTagPose.minus(
            new Pose2d(
              new Translation2d(distanceFromAprilTag, 0).rotateBy(targetAprilTagPose.getRotation().plus(
                new Rotation2d(Units.degreesToRadians(-180)))
              ),
              new Rotation2d()
            )
          )
        ).plus(new Transform2d(0, 0, new Rotation2d(Units.degreesToRadians(180))))
      );
    };
    /*public Command driveToAprilTag(int apriltagNunber, double distanceFromAprilTag){
      return driveToAprilTag(apriltagNunber, distanceFromAprilTag, new Rotation2d());
    }
    public Command driveToAprilTag(int apriltagNunber, double rotationOffset){
      return driveToAprilTag(apriltagNunber, 1, rotationOffset);
    }*/
    public Command driveToAprilTag(int apriltagNunber){
      return swerveSubsystem.driveToPose(
        aprilTagFieldLayout.getTagPose(1).get().toPose2d()
      );
    }

  public Command driveToReef(int apriltagnumber, boolean left){
    Pose2d targetAprilTagPose = aprilTagFieldLayout.getTagPose(apriltagnumber).get().toPose2d();
    int distanceToSide = 1;
    return swerveSubsystem.driveToPose(
      //targetAprilTagPose.plus(new Transform2d(new Translation2d(2, 0).rotateBy(targetAprilTagPose.getRotation()),new Rotation2d()))
      new Pose2d().transformBy(
        targetAprilTagPose.minus(
          new Pose2d(
            new Translation2d(swerveSubsystem.getSwerveDrive().swerveDriveConfiguration.getDriveBaseRadiusMeters(), .5).rotateBy(targetAprilTagPose.getRotation().plus(
              new Rotation2d(Units.degreesToRadians(-180)))
            ),
            new Rotation2d()
          )
        )
      ).plus(new Transform2d(0, 0, new Rotation2d(Units.degreesToRadians(180))))
    );
  };
}
