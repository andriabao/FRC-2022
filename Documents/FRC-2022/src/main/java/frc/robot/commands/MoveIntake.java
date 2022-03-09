package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Control;
import frc.robot.Robot;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.BallHandler.BallHandlerState;

public class MoveIntake extends CommandBase {

    // Compressor compressor = new Compressor(11, PneumaticsModuleType.CTREPCM);

    public MoveIntake() {
        addRequirements(Robot.ballHandler);
    }

    public void initialize() {
        Robot.ballHandler.state = BallHandlerState.MOVE;
        // compressor.enableDigital();
    }
    

    public void execute() {
        Robot.ballHandler.isIntakeExtended = Control.getInstance().getToggleIntake();
        Robot.ballHandler.state = BallHandlerState.MOVE;
    }

    public void end(boolean interrupted) {
        Robot.ballHandler.state = BallHandlerState.IDLE;
    }

    
}
