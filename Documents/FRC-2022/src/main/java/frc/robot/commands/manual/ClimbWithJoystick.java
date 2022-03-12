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
    // TODO limit and overrides
    boolean override = false;
    if(Robot.climber.leftEncoder.getPosition() <= Robot.climber.leftZero 
    && Robot.climber.left.get() < 0){
      Robot.climber.left.set(0);
      override = true;
    }
    if(Robot.climber.leftEncoder.getPosition() >= Robot.climber.leftZero + 180 
    && Robot.climber.left.get() > 0){
      Robot.climber.left.set(0);
      override = true;
    }
    if(Robot.climber.rightEncoder.getPosition() <= Robot.climber.rightZero 
    && Robot.climber.right.get() < 0){
      Robot.climber.right.set(0);
      override = true;
    }
    if(Robot.climber.rightEncoder.getPosition() >= Robot.climber.rightZero + 180 
    && Robot.climber.right.get() > 0){
      Robot.climber.right.set(0);
      override = true;
    }

    if(!override) {
      Robot.climber.left.set(control.getLeftWinchSpeed());
      Robot.climber.right.set(control.getRightWinchSpeed());  
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
