package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Control;
import frc.robot.Robot;

public class ClimbWithJoystick extends CommandBase {
  private Control control = Control.getInstance();

  public ClimbWithJoystick() {
    addRequirements(Robot.climber);
  }

  @Override
  public void execute() {
    double leftSpeed = control.getLeftWinchSpeed();
    double rightSpeed = control.getRightWinchSpeed();
    if (!control.isOverride()) {
      if(Robot.climber.leftEncoder.getPosition() <= Robot.climber.leftZero 
      && leftSpeed < 0){
        leftSpeed = 0;
      }
      if(Robot.climber.leftEncoder.getPosition() >= Robot.climber.leftZero + 180 
      && leftSpeed > 0){
        leftSpeed = 0;
      }
      if(Robot.climber.rightEncoder.getPosition() <= Robot.climber.rightZero 
      && rightSpeed < 0){
        rightSpeed = 0;
      }
      if(Robot.climber.rightEncoder.getPosition() >= Robot.climber.rightZero + 180 
      && rightSpeed > 0){
        rightSpeed = 0;
      }
    }
    
    Robot.climber.left.set(leftSpeed);
    Robot.climber.right.set(rightSpeed);  
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
