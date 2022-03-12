package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team254.lib.util.MinTimeBoolean;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Control;


public class BallHandler extends SubsystemBase {
  public enum BallHandlerState {
    IDLE,
    MOVE,
    INTAKE,
    PRESPIN,
    SHOOT,
    EJECT,
  }
  public BallHandlerState state = BallHandlerState.IDLE;
  public int ballCnt = 0;
  public double desiredRPM;
  public int isIntakeExtended;
  
  public enum ShootingState {
    HOLD,
    PRESHOOT,
    SHOOT
  }
  private BallHandlerState lastState = state;
  public ShootingState shootingState = ShootingState.HOLD;
  // shooter
  private CANSparkMax shooterMaster = new CANSparkMax(20, MotorType.kBrushless);
  private CANSparkMax shooterSlave = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax shooterConveyer = new CANSparkMax(12, MotorType.kBrushless);
  public RelativeEncoder encoder;
  private SparkMaxPIDController shooterPIDController;
  private DigitalInput shooterBeamBreaker = new DigitalInput(2);
  private DigitalInput indexBeamBreaker = new DigitalInput(1);
  // auto
  private MinTimeBoolean isHoldingForLastBall = new MinTimeBoolean(Constants.MIN_SHOOT_GAP_TIME);
  private TimeDelayedBoolean isFreeSpinning = new TimeDelayedBoolean(Constants.MAX_SHOOTER_FREE_SPIN_TIME);
  // intake
  private CANSparkMax ballIntake = new CANSparkMax(10, MotorType.kBrushless);
  private DigitalInput intakeBeamBreaker = new DigitalInput(0);
  //pneumatics
  Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

  private static final boolean BALL = false, NO_BALL = true;
  
  private boolean lastIntakeBeam, lastShooterBeam;

  public BallHandler() {
    lastIntakeBeam = intakeBeamBreaker.get();
    lastShooterBeam = shooterBeamBreaker.get();

    setSpark(shooterMaster);
    setSpark(shooterSlave);
    setSpark(shooterConveyer);
    setSpark(ballIntake);

    ballIntake.setInverted(false);
    ballIntake.setSmartCurrentLimit(30);
    ballIntake.burnFlash();
    
    shooterMaster.setInverted(true);
    shooterMaster.setClosedLoopRampRate(0.3);
    // shooterMaster.setSmartCurrentLimit(30);
    // shooterSlave.setSmartCurrentLimit(30);
    // burning flash somehow causes bug
    shooterSlave.follow(shooterMaster, true);
    
    shooterConveyer.setInverted(false);
    shooterConveyer.setIdleMode(IdleMode.kBrake);
    shooterConveyer.setSmartCurrentLimit(20);
    shooterConveyer.burnFlash();

    encoder = shooterMaster.getEncoder();
    shooterPIDController = shooterMaster.getPIDController();
    setShooterPID();
  }

  @Override
  public void periodic() {
    // System.out.println(ballIntake.getOutputCurrent());
    // desiredRPM = Control.getInstance().getSlider() * 1500 + 4100;
    switch (state) {
      case IDLE:
        ballIntake.set(0);
        shooterConveyer.set(0);
        shooterMaster.set(0);
        isFreeSpinning.update(false);
        break;
      
      case MOVE:
        ballIntake.set(0);
        shooterConveyer.set(0);
        shooterMaster.set(0);
        isFreeSpinning.update(false);        
        if(isIntakeExtended == 1) {
          leftSolenoid.set(true);
          rightSolenoid.set(true);
        } else if(isIntakeExtended == -1) {
          leftSolenoid.set(false);
          rightSolenoid.set(false);
        }

      case PRESPIN:
        ballIntake.set(0);
        shooterMaster.set(0);
        // shooterConveyer.set(shooterBeamBreaker.get() == NO_BALL? 1 : 0);
        shooterPIDController.setReference(desiredRPM, ControlType.kVelocity,
            0, Constants.SHOOTER_KS);
        isFreeSpinning.update(false);
        break;

      case SHOOT:
        
        // if we just enter shooting state, hold for RPM to ramp up
        if (lastState != BallHandlerState.SHOOT)
          shootingState = ShootingState.HOLD;
        // if a ball just got shot, wait for RPM to get back
        if (lastShooterBeam == BALL && shooterBeamBreaker.get() == NO_BALL) {
          shootingState = ShootingState.HOLD;
          isHoldingForLastBall.update(true);
          isFreeSpinning.update(false);
        }
        // we start shooting only if there is enough gap from last shot and 
        // and that the wheel spins fast enough
        // or if desiredRPM is unattainable after free spinning for a while
        if (isHoldingForLastBall.get() == false && 
            Math.abs(desiredRPM - encoder.getVelocity()) < Constants.MAX_SHOOT_RPM_ERROR
            || isFreeSpinning.get())
          shootingState = ShootingState.SHOOT;

        ballIntake.set(0);
        shooterConveyer.set(shootingState == ShootingState.SHOOT? 1 : 0);
        shooterPIDController.setReference(desiredRPM, ControlType.kVelocity,
            0, Constants.SHOOTER_KS);
        
        isFreeSpinning.update(true);     

        break;

      case INTAKE:
        ballIntake.set(1);

        if(indexBeamBreaker.get() == NO_BALL) {
          shooterConveyer.set(1);
        } else {
          shooterConveyer.set(0);
        }

        shooterMaster.set(0);
        isFreeSpinning.update(false);
        break;
      
      case EJECT:
        
        if(Control.getInstance().isOverride() == false) {
          ballIntake.set(1);
          shooterConveyer.set(1);
          shooterPIDController.setReference(800, ControlType.kVelocity,
              0, 1);

        } 
        if(Control.getInstance().isOverride() == true) {
          ballIntake.set(-1);
          shooterConveyer.set(-1);
        }

        break;
    }
    lastState = state;

    updateBallCnt();
    SmartDashboard.putNumber("ball_handler/cnt", ballCnt);
    SmartDashboard.putString("ball_handler/state", state.toString());
    SmartDashboard.putNumber("ball_handler/shooter_rpm", encoder.getVelocity());
    SmartDashboard.putNumber("ball_handler/desired_rpm", desiredRPM);
    SmartDashboard.putString("ball_handler/intakeBeam", 
      intakeBeamBreaker.get() == BALL? "BALL" : "NO_BALL");
    SmartDashboard.putString("ball_handler/shooterBeam", 
      shooterBeamBreaker.get() == BALL? "BALL" : "NO_BALL");
  }

  private void updateBallCnt() {
    if (lastIntakeBeam == NO_BALL && intakeBeamBreaker.get() == BALL &&
        state != BallHandlerState.EJECT)
      ballCnt ++;
    else if (lastIntakeBeam == BALL && intakeBeamBreaker.get() == NO_BALL &&
        state == BallHandlerState.EJECT)
      ballCnt --;
    lastIntakeBeam = intakeBeamBreaker.get();

    if (lastShooterBeam == BALL && shooterBeamBreaker.get() == NO_BALL)
      ballCnt --;
    lastShooterBeam = shooterBeamBreaker.get();
    
    if(ballCnt > 5) 
      ballCnt = 5;
    if(ballCnt < 0)
      ballCnt = 0;
  }

  private void setSpark(CANSparkMax spark) {
    spark.restoreFactoryDefaults();
  }

  private void setShooterPID(){
    shooterPIDController.setP(Constants.SHOOTER_V_GAINS.kP);
    shooterPIDController.setI(Constants.SHOOTER_V_GAINS.kI);
    shooterPIDController.setFF(Constants.SHOOTER_V_GAINS.kF);
    shooterPIDController.setD(Constants.SHOOTER_V_GAINS.kD);
    shooterPIDController.setOutputRange(-1, 1);
  }
}
