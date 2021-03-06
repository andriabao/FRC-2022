package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Coprocessor extends SubsystemBase {
  //public NetworkTable odomTable = NetworkTableInstance.getDefault().getTable("odom");
  public NetworkTable odomTable = NetworkTableInstance.getDefault().getTable("LiveWindow/ShooterCamera");
  public NetworkTable ballTable = NetworkTableInstance.getDefault().getTable("LiveWindow/IntakeCamera");

  // flags coresponding to connection of Nano, tracking camera, RGB camera
  public boolean isConnected;
  // target
  public double targetFieldTheta, targetRelativeDirLeft, targetDis, yMin, bestFit;
  public double isTargetFound;

  public boolean isTargetGood;
  // pose
  public double fieldX, fieldY, fieldTheta;
  public boolean isPoseGood;
  // ball
  public double ballDis, ballFieldTheta, ballRelativeDirLeft, ballFound, lastUpdate;
  public boolean isBallGood, isBallFound;
  public double innerDis, innerAngleDelta;

  private double lastClientTime;
  private int disconnectCnt;

  public Coprocessor() {
  }

  @Override
  public void periodic() {
    checkConnection();
    // pose
    fieldX = odomTable.getEntry("field_x").getDouble(0);
    fieldY = odomTable.getEntry("field_y").getDouble(0);
    fieldTheta = odomTable.getEntry("field_t").getDouble(0);
    yMin = odomTable.getEntry("y_min").getDouble(0);

    // target
    isTargetFound = odomTable.getEntry("goal_detected").getDouble(0); //
    targetRelativeDirLeft = odomTable.getEntry("x_angle").getDouble(0); //
    
    targetFieldTheta = odomTable.getEntry("x_angle").getDouble(0); //
    targetDis = odomTable.getEntry("distance").getDouble(0); //
    bestFit = odomTable.getEntry("line_best_fit").getDouble(0);
   
    // ball
    ballFound = ballTable.getEntry("ball_detected").getDouble(0); //
    if(ballFound == 0) {
      isBallFound = false;
    } else {
      isBallFound = true;
    }

    ballDis = ballTable.getEntry("ball_distance").getDouble(0); //
    ballRelativeDirLeft = ballTable.getEntry("ball_angle").getDouble(0); //
    ballFieldTheta = ballTable.getEntry("ball_angle").getDouble(0); //
    lastUpdate = ballTable.getEntry("last_update_time").getDouble(0);
    // solve the triangle between robot, outer target, and inner target
    if (isFieldCalibrated() && isTargetGood && isPoseGood && isTargetFound ==1) {
      // law of cos
      innerDis = Math.sqrt(Math.max(0, 
          0.74 * 0.74 + targetDis * targetDis - 
          1.48 * targetDis * Math.cos(Math.toRadians(-targetFieldTheta))));
      // law of sine
      innerAngleDelta = Math.toDegrees(Math.asin(0.74 * 
          Math.sin(Math.toRadians(-targetFieldTheta)) / innerDis));
      if (Math.abs(innerAngleDelta) > 1)
        innerAngleDelta = Math.signum(innerAngleDelta) * 1;
      // actual_target = target_theta - delta
      // System.out.printf("%.2f %.2f\n", innerDis, innerAngleDelta); 
      odomTable.getEntry("inner_target_dis").setDouble(innerDis);
    } else {
      innerDis = targetDis + 0.3;
    }
    // opposite target detection
    if(isTargetFound==1 && isPoseGood && isFieldCalibrated() && 
      Math.abs((Math.toDegrees(Math.atan2(4.4, -15.98)) + 180 - targetFieldTheta)
      % 360) < 15) {
      System.out.println("wrong target");
      isTargetFound = 0;
      targetFieldTheta = Math.atan2(fieldY, fieldX) + 180;
    }
  }

  /** This method updates if Nano is working as expected
   */
  private void checkConnection() {
    double clientTime = odomTable.getEntry("client_time").getDouble(0);
    if(clientTime == lastClientTime){
      disconnectCnt++;
      if(disconnectCnt > 15){
        isConnected=false;
        // odomTable.getEntry("field_calibration_good").setBoolean(false);
      }
    }
    else{
      disconnectCnt=0;
      isConnected=true;
    }
    lastClientTime = clientTime;

    isTargetGood = odomTable.getEntry("target_good").getBoolean(false);
    isPoseGood = odomTable.getEntry("pose_good").getBoolean(false);
    isBallGood = odomTable.getEntry("ball_good").getBoolean(false);
    NetworkTableInstance.getDefault().getEntry("/coprocessor/working").
        setBoolean(isConnected && isPoseGood && isTargetGood && isBallGood);
    String error = "";
    if (!isConnected)
      error = "COMM";
    else {
      if (!isTargetGood)
        error += "CV|";
      if (!isPoseGood)
        error += "POSE|";
      if (!isBallGood)
        error += "BALL";
    } 
    NetworkTableInstance.getDefault().getEntry("/coprocessor/error").
        setString(error);
  }

  public boolean isFieldCalibrated() {
    return odomTable.getEntry("field_calibration_good").getBoolean(false);
  }

  public void calibrate_field() {
    odomTable.getEntry("field_calibration_start").setBoolean(true);
  }

  public Pose2d getPose() {
    return new Pose2d(fieldX, fieldY, new Rotation2d(Math.toRadians(fieldTheta)));
  }
}
