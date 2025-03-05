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
    /* */
    /*
      Pose2d targetAprilTagPose = aprilTagFieldLayout.getTagPose(apriltagnumber).get().toPose2d();
      System.out.println("----------------------------------------");
      System.out.println(targetAprilTagPose);
      System.out.println(
          new Translation2d(.25, targetAprilTagPose.getRotation())
      );
      System.out.println(
        new Transform2d(
          new Translation2d(.25, targetAprilTagPose.getRotation()),
          new Rotation2d(Units.degreesToRadians(-180))
        )
      );
      System.out.println(
        targetAprilTagPose.plus(new Transform2d(
          new Translation2d(.25, targetAprilTagPose.getRotation()),
          new Rotation2d(Units.degreesToRadians(-180))
        )
        )
      );
      System.out.println(
      new Pose2d(3.66, 4.03,new Rotation2d(Units.degreesToRadians(180))).plus(
        new Transform2d(
          new Translation2d(.25, targetAprilTagPose.getRotation()),
          new Rotation2d(Units.degreesToRadians(-180))
        )
        )
      );
      System.out.println(
        new Pose2d(1,1,new Rotation2d()).plus(
          new Transform2d(
            new Translation2d(1,targetAprilTagPose.getRotation()),
            new Rotation2d(Units.degreesToRadians(-180)))
          )
        );
      System.out.println("----------------------------------------");




      /*
      System.out.println(3.66-.5);
      System.out.println("-----------------------------------------");
      System.out.println(targetAprilTagPose);
      System.out.println(new Translation2d(.5, targetAprilTagPose.getRotation()));
      System.out.println(
        new Transform2d(
          new Translation2d(.25, targetAprilTagPose.getRotation()),
          new Rotation2d(Units.degreesToRadians(-180))
        )
      );
      System.out.println(
        targetAprilTagPose.plus(
          new Transform2d(
            new Translation2d(.25, targetAprilTagPose.getRotation()),
            new Rotation2d(Units.degreesToRadians(-180))
          )
        )
      );
      System.out.println("-----------------------------------------");

      System.out.println(new Pose2d(3.66,4.03,new Rotation2d()));
      System.out.println(new Translation2d(.5, targetAprilTagPose.getRotation()));
      System.out.println(
        new Transform2d(
          new Translation2d(.5, targetAprilTagPose.getRotation()),
          new Rotation2d(Units.degreesToRadians(-180))
        )
      );
      System.out.println(
        new Pose2d(3.66,4.03,new Rotation2d()).plus(
          new Transform2d(
            new Translation2d(.5, targetAprilTagPose.getRotation()),
            new Rotation2d(Units.degreesToRadians(-180))
          )
        )
       );

      //System.out.println(targetAprilTagPose);
      //System.out.println(new Translation2d(3, targetAprilTagPose.getRotation()));
      //System.out.println(new Transform2d(new Translation2d(3, targetAprilTagPose.getRotation()),new Rotation2d()));
      /*
      System.out.println(
        new Pose2d(
          new Translation2d(-.5, targetAprilTagPose.getRotation().times(-1)),
          new Rotation2d(Units.degreesToRadians(180))
        )
      );*/
      /*System.out.println(
        new Transform2d(
          new Translation2d(.5, targetAprilTagPose.getRotation()),
          new Rotation2d(Units.degreesToRadians(-180))
        )
      );
      System.out.println(
        new Pose2d(
          new Translation2d(.5, targetAprilTagPose.getRotation()).times(-1),
          new Rotation2d(Units.degreesToRadians(180))
        )
      );
      System.out.println(
        targetAprilTagPose.minus(
          new Pose2d(
            new Translation2d(.5, targetAprilTagPose.getRotation()).times(-1),
            new Rotation2d(Units.degreesToRadians(180))
          )
        )
      );
      System.out.println(
        new Pose2d(
          new Translation2d(.5, 0).rotateBy(targetAprilTagPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-180)))),
          new Rotation2d()
        )
      );
      System.out.println(
        targetAprilTagPose.minus(
          new Pose2d(
            new Translation2d(1, 0).rotateBy(targetAprilTagPose.getRotation().plus(
                new Rotation2d(Units.degreesToRadians(-180))
              )
            ),
            new Rotation2d()
          )
        )
      );
      System.out.println(
        targetAprilTagPose.minus(
          new Pose2d(
            new Translation2d(1, 0).rotateBy(targetAprilTagPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-180)))),
            new Rotation2d())
          )
        );*/
      
      //System.out.println();
      //System.out.println(targetAprilTagPose.minus(new Pose2d(new Translation2d(2, 0).rotateBy(targetAprilTagPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-180)))),new Rotation2d())));
      

      addCommands(
        //swerveSubsystem.driveToPose(
        //  //targetAprilTagPose.plus(new Transform2d(new Translation2d(2, 0).rotateBy(targetAprilTagPose.getRotation()),new Rotation2d()))
        //  new Pose2d().transformBy(targetAprilTagPose.minus(new Pose2d(new Translation2d(distanceFromAprilTag, 0).rotateBy(targetAprilTagPose.getRotation().plus(new Rotation2d(Units.degreesToRadians(-180)))),new Rotation2d())))
        //),S
        swerveSubsystem.driveToAprilTag(apriltagnumber, 1.2),
        swerveSubsystem.driveToReef(apriltagnumber, true),
        swerveSubsystem.driveToAprilTag(13,swerveSubsystem.getSwerveDrive().swerveDriveConfiguration.getDriveBaseRadiusMeters())
        
        //swerveSubsystem.driveToPose(aprilTagFieldLayout.getTagPose(12).get().toPose2d())
      );
  }
}
