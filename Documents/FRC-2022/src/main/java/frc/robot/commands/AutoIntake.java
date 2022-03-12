package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Control;
import frc.robot.Robot;
import frc.robot.subsystems.Coprocessor;
import frc.robot.subsystems.BallHandler.BallHandlerState;

public class AutoIntake extends CommandBase {
  private static final double angleP = 1.0 / 40.0;

  private boolean isCanceled, quitWithoutBall;
  private int targetBallCnt;
  private Control control = Control.getInstance();
  double minLinearSpeed = 1;


  private PIDController angularPIDController = new PIDController(Constants.TURNING_GAINS.kP,
  Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

  private PIDController linearPIDController = new PIDController(Constants.DRIVETRAIN_DISTANCE_GAINS.kP,
  Constants.DRIVETRAIN_DISTANCE_GAINS.kI, Constants.DRIVETRAIN_DISTANCE_GAINS.kD);

  double angularSetpoint, linearSetpoint, angularUpdateTime, linearUpdateTime, prevUpdateTime;

  boolean angularComplete, linearComplete;
  // private PIDController pidController = new PIDController(Constants.TURNING_GAINS.kP,
  // Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

  public AutoIntake(int _targetBallCnt, boolean _quiteWithoutBall) {
    targetBallCnt = _targetBallCnt;
    quitWithoutBall = _quiteWithoutBall;
    addRequirements(Robot.driveSubsystem);
    addRequirements(Robot.ballHandler);
  }

  public AutoIntake() {
    this(5, false);
  }

  @Override
  public void initialize() {
    angularPIDController.reset();
    linearPIDController.reset();

    angularSetpoint = Robot.driveSubsystem.getHeading() + Robot.coprocessor.ballFieldTheta;
    linearSetpoint = Robot.driveSubsystem.getAvgEncoderDistance() + Robot.coprocessor.ballDis;

    angularUpdateTime = Timer.getFPGATimestamp() + 2;
    linearUpdateTime = Timer.getFPGATimestamp() + 2;

    angularComplete = false;
    linearComplete = false;
    prevUpdateTime = Robot.coprocessor.lastUpdate;
  }

  public void execute() {

    Robot.ballHandler.state = BallHandlerState.INTAKE;

    if(prevUpdateTime != Robot.coprocessor.lastUpdate && Timer.getFPGATimestamp() > angularUpdateTime && angularComplete){
      angularSetpoint = Robot.driveSubsystem.getHeading() + Robot.coprocessor.ballFieldTheta;
      angularUpdateTime = Timer.getFPGATimestamp() + 2;
      angularComplete = false;
      prevUpdateTime = Robot.coprocessor.lastUpdate;
      System.out.println("update anuglar");
    }

    // if(Robot.coprocessor.isBallFound && Timer.getFPGATimestamp() > linearUpdateTime && linearComplete){
    //   linearSetpoint = Robot.driveSubsystem.getAvgEncoderDistance() + (Robot.coprocessor.ballDis);
    //   linearUpdateTime = Timer.getFPGATimestamp() + ;
    //   linearComplete = false;
    //   System.out.println("update linear");
    // }

    // System.out.println(drive.getAvgEncoderDistance() + " " + linearSetpoint + " " + linearComplete);


    // ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(0, 0, v_angular);

    // DifferentialDriveWheelSpeeds wheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(adjustedSpeeds);

    double leftFF = Constants.ks;
    double rightFF = Constants.ks; 

    double angularPID;
    double linearPID;

    if(Math.abs(Robot.driveSubsystem.getHeading() - angularSetpoint) <= 1){
      angularPID = 0;
      angularComplete = true;
    }else{
      angularPID = angularPIDController.calculate(Robot.driveSubsystem.getHeading(), angularSetpoint);
    }

    if(Math.abs(Robot.driveSubsystem.getAvgEncoderDistance() - linearSetpoint) <= 0.05){
      linearPID = 0;
      linearComplete = true;
      // System.out.println("linear complete");
    }else{
      linearPID = linearPIDController.calculate(Robot.driveSubsystem.getAvgEncoderDistance(), linearSetpoint);
    }

    if(Robot.coprocessor.isBallFound && Math.abs(linearPID)< minLinearSpeed){
      linearPID = Math.signum(minLinearSpeed);
    }

    // double left = linearPID + angularPID;
    // double right = linearPID - angularPID;

    // double left = linearPID;
    // double right = linearPID;

    double left = angularPID;
    double right = - angularPID;

    if(Math.abs(left) > Constants.kMaxSpeed_Drive){
      left = Constants.kMaxSpeed_Drive * Math.signum(left);
      right = right * Constants.kMaxSpeed_Drive / Math.abs(left);
    }

    if(Math.abs(right) > Constants.kMaxSpeed_Drive){
      right = Constants.kMaxSpeed_Drive * Math.signum(right);
      left = left * Constants.kMaxSpeed_Drive / Math.abs(right);
    }

    double linearSpeed = 0;
    double ballDis = Robot.coprocessor.ballDis;
    if(!Robot.coprocessor.isBallFound)
      ballDis = 0;
    linearSpeed = 0.2 + 0.6 * (ballDis);
    
    // Robot.driveSubsystem.setVelocity(linearSpeed, 
    //                                  linearSpeed);
    NetworkTableInstance.getDefault().getEntry("/drivetrain/auto_state").setString(
        "AUTO_INTAKE_BALL");

    Robot.driveSubsystem.setVelocity(leftFF * Math.signum(left) + left + linearSpeed, 
    rightFF * Math.signum(right) + right + linearSpeed);
    
    System.out.println(left + " " + right);
  }


  @Override
  public void end(boolean interrupted) {
    Robot.ballHandler.state = BallHandlerState.IDLE;
    NetworkTableInstance.getDefault().getEntry("/drivetrain/auto_state").setString(
        "MANUAL");
  }

  @Override
  public boolean isFinished() {
    if(Robot.coprocessor.ballDis <= 1 && Math.abs(Robot.coprocessor.ballFieldTheta) <= 10){
      return true;
    }
    return false;
  //   if (isCanceled)
  //     return true;
  //   boolean ballFound = Robot.coprocessor.isConnected && Robot.coprocessor.isBallGood && 
  //       Robot.coprocessor.isBallFound;
  //   if (quitWithoutBall && !ballFound)
  //     return true;
  //   return !control.isOverrideAutoIntake() && Robot.ballHandler.ballCnt >= targetBallCnt;
  }
}
