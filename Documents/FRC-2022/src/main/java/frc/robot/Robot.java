package frc.robot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.team254.lib.util.DriveSignal;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.Auto.*;
import frc.robot.commands.manual.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.BallHandler.BallHandlerState;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Robot extends TimedRobot {
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static BallHandler ballHandler = new BallHandler();
  public static Coprocessor coprocessor = new Coprocessor();
  public static Climber climber = new Climber();

  public static ExecutorService cocurrentExecutor = 
      Executors.newFixedThreadPool(1);
  
  public static double matchStartTime;

  public static DriveWithJoystick driveWithJoystick = new DriveWithJoystick();

  private static Control control = Control.getInstance();
  private static AutoShoot autoShoot = new AutoShoot(false);
  private static ManualShoot manualShoot = new ManualShoot(3);
  private static AutoIntake autoIntake = new AutoIntake();
  private static ManualIntake manualIntake = new ManualIntake();
  private static Eject eject = new Eject();
  private static MoveIntake moveIntake = new MoveIntake();
  private static UsbCamera camera = null;
  private static final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public CANSparkMax leftMaster, leftSlave1, leftSlave2;

  @Override
  public void robotInit() {
    driveSubsystem.setDefaultCommand(driveWithJoystick);
    ballHandler.setDefaultCommand(new HandleBallWithJoystick());
    climber.setDefaultCommand(new ClimbWithJoystick());

    autonomousChooser.setDefaultOption("seven_trench_stable", new SevenBallStable());
    autonomousChooser.addOption("steal_one_seven_ball", new StealOneSevenBall());
    autonomousChooser.addOption("six_trench", new SixBall());
    autonomousChooser.addOption("six_bar", new SixBallBar());
    // autonomousChooser.addOption("move_forward", new DriveUntil(1).withTimeout(2));
    SmartDashboard.putData("Auto choices", autonomousChooser);

    // leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    // setSpark(leftMaster);


  }

  private void setSpark(final CANSparkMax spark) {
    spark.restoreFactoryDefaults();
    spark.setOpenLoopRampRate(0.4);
    spark.setClosedLoopRampRate(0.4);
    // spark.enableVoltageCompensation(12.0);
    spark.setSmartCurrentLimit(40);
  }

  @Override
  public void autonomousInit() {
    // Robot.ballHandler.ballCnt = 3;
    // matchStartTime = Timer.getFPGATimestamp();

    // while (coprocessor.isConnected 
    //        && !coprocessor.isFieldCalibrated()
    //        && coprocessor.isPoseGood
    //        && coprocessor.isTargetGood
    //        && Timer.getFPGATimestamp() - matchStartTime < 3) {
    //   coprocessor.calibrate_field();
    //   Timer.delay(0.02);
    // }
    // if (!coprocessor.isConnected || !coprocessor.isTargetGood) {
    //           System.out.println("emergency move");
    //           new SequentialCommandGroup(
    //             new ManualShoot(3).withTimeout(4),
    //             new DriveUntil(1).withTimeout(3)
    //           ).schedule();
    // } else if (!coprocessor.isPoseGood) {
    //   new SixBall().schedule();
    // } else {
      autonomousChooser.getSelected().schedule();
      // new SequentialCommandGroup(
      //                new MoveIntake().withTimeout(1),
      //                new DriveUntil(1).withTimeout(0.5),
      //                new ManualIntake().withTimeout(1.5),
      //                new AutoShoot().withTimeout(4),
      //                new DriveUntil(0.5, -0.5, () -> Robot.coprocessor.isBallFound).withTimeout(1.55),
      //                new DriveUntil(0).withTimeout(1.5),
      //                new AutoIntake().withTimeout(5),
      //                new DriveUntil(-0.7, 0.7, () -> Robot.coprocessor.isTargetFound == 1).withTimeout(1),
      //                new AutoShoot().withTimeout(4)

      //              ).schedule();
      //   new DriveUntil(1).withTimeout(3)
    // }
  }

  /* RobotPeriodic is called after the coresponding periodic of the stage,
  *  such as teleopPeriodic
  */ 
  @Override
  public void robotPeriodic() {
    // sequence of running: subsystems, buttons, commands

    // double speed = 1.0;
    // leftMaster.set(speed);

    String streamMode = "";
    if (ballHandler.state == BallHandlerState.SHOOT || ballHandler.state == BallHandlerState.PRESPIN)
      streamMode = "shoot";
    else if (ballHandler.state == BallHandlerState.INTAKE)
      streamMode = "intake";
    else
      streamMode = control.isReversed()? "shoot":"intake";
    NetworkTableInstance.getDefault().getEntry("/odom/video_output").setString(streamMode);
    
    if (control.isCalibrateField())
      coprocessor.calibrate_field();

    CommandScheduler.getInstance().run();
    NetworkTableInstance.getDefault().flush();


  }

  @Override
  public void teleopPeriodic() {
    if(Control.getInstance().isEStop()){
      // make sure to use wpilib above 2020.2, otherwise there would be a bug
      CommandScheduler.getInstance().cancelAll();
      ballHandler.state = BallHandlerState.IDLE;
      driveSubsystem.setOpenLoop(new DriveSignal(0, 0, true));
      return;
    }

    // ballHandler
    if (control.isResetBallCnt())
      ballHandler.ballCnt = 0;
    if (control.isChangeInner())
      autoShoot.innerGoal ^= true;
    SmartDashboard.putBoolean("isInner", autoShoot.innerGoal);
    
    if (control.isAutoShoot() || control.isOverrideAutoShoot() && Robot.coprocessor.isTargetFound == 1)
      autoShoot.schedule();
    else if (control.isAutoIntake() || control.isOverrideAutoIntake())
      autoIntake.schedule();
    else if (control.isManualShoot())
      manualShoot.schedule();
    else if(control.isManualIntake() || control.isOverrideManualIntake())
      manualIntake.schedule();
    else if(control.isEject())
      eject.schedule();
    else if(control.isToggleIntake())
      moveIntake.schedule();

    if (control.getLeftWinchSpeed() > 0.1 &&  camera == null) {
      camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setFPS(20);
    }
    
  }
}
