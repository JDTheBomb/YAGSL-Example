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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagPathPlannerAuto extends SequentialCommandGroup {
  /** Creates a new AprilTagPathPlannerAuto. */
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public AprilTagPathPlannerAuto(SwerveSubsystem swerveSubsystem, int apriltagnumber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        swerveSubsystem.driveToAprilTag(apriltagnumber, new Translation2d(1,0)),
        swerveSubsystem.driveToReef(apriltagnumber, true),
        swerveSubsystem.driveToAprilTag(13,0)
        
        //swerveSubsystem.driveToPose(aprilTagFieldLayout.getTagPose(12).get().toPose2d())
      );
  }
}
