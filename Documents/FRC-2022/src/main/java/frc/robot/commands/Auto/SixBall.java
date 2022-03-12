package frc.robot.commands.Auto;

import frc.robot.Robot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveUntil;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.manual.ManualIntake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SixBall extends SequentialCommandGroup {
  public SixBall() {
    addCommands(new MoveIntake().withTimeout(0.3));
    addCommands(new DriveUntil(1).withTimeout(0.5));
    addCommands(new ManualIntake().withTimeout(1.5));
    addCommands(new AutoShoot().withTimeout(2.5));
  }
}
