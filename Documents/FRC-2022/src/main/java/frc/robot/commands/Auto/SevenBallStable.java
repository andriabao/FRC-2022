package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveUntil;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.manual.ManualIntake;

public class SevenBallStable extends SequentialCommandGroup {
  public SevenBallStable() {
    addCommands(new MoveIntake().withTimeout(1.2));
    addCommands(new DriveUntil(1).withTimeout(0.5));
    addCommands(new ManualIntake().withTimeout(1.5));
    addCommands(new AutoShoot().withTimeout(2.5));
    addCommands(new DriveUntil(1, -1, () -> Robot.coprocessor.isBallFound).withTimeout(0.6));
    addCommands(new DriveUntil(0).withTimeout(1.4));
    addCommands(new AutoIntake().withTimeout(3.5));
    addCommands(new DriveUntil(-0.7, 0.7, () -> Robot.coprocessor.isTargetFound == 1).withTimeout(1));
    addCommands(new AutoShoot().withTimeout(4));
    addCommands(new AutoIntake().withTimeout(6));
    // addCommands(new DriveUntil(-3).withTimeout(1));
    addCommands(new AutoShoot().withTimeout(4));


  }
}
