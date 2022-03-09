package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;

public class CriticalPoints {
  // All points follow WPI convention with origin at our target

  public static final Pose2d ourTrenchRunPreEntry = 
      new Pose2d(-2, -1.2, new Rotation2d(Math.toRadians(180)));
  public static final Pose2d ourTrenchRunEntry = 
      new Pose2d(-4.7, -1.7, new Rotation2d(Math.toRadians(180)));
  public static final Pose2d ourTrenchRunThreeBallStop = 
      new Pose2d(-8, -1.7, new Rotation2d(Math.toRadians(180)));

//   public static final Pose2d barThreeBallSideEntry = 
//       new Pose2d(-6.60, 3.26, new Rotation2d(Math.toRadians(-67.5)));
public static final Translation2d barThreeBallSidePreEntry = 
    new Translation2d(-5, 2.8);
  public static final Pose2d barThreeBallSideEntry = 
        new Pose2d(-6.30, 2.5, new Rotation2d(Math.toRadians(-67.5)));
//   public static final Pose2d barThreeBallSideStop = 
//       new Pose2d(-5.78, 1.28, new Rotation2d(Math.toRadians(-67.5)));

  public static final Pose2d barTwoBallSideEntry = 
      new Pose2d(-5.15, 1.00, new Rotation2d(Math.toRadians(22.5)));
  public static final Pose2d barTwoBallSideStop = 
      new Pose2d(-5.97, 0.66, new Rotation2d(Math.toRadians(22.5)));
  
  public static Trajectory trenchPickUpThreeTrajectory;

  public static final Pose2d shootPoint = new Pose2d(-5, -1, new Rotation2d(Math.toRadians(190)));
}
