package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // 1 is pull
  public CANSparkMax left = new CANSparkMax(30, MotorType.kBrushless);
  // 1 is up
  public CANSparkMax right = new CANSparkMax(31, MotorType.kBrushless);
  public double leftZero, rightZero;
  public RelativeEncoder leftEncoder, rightEncoder;


  
  public Climber() {
    setSpark(left);
    left.setSmartCurrentLimit(40);
    setSpark(right);
    right.setSmartCurrentLimit(40);

    leftEncoder = left.getEncoder();
    leftEncoder.setVelocityConversionFactor(1);

    rightEncoder = right.getEncoder();
    rightEncoder.setVelocityConversionFactor(1);

    setZero();

  }

  public void setZero(){
    leftZero = leftEncoder.getPosition();
    rightZero = rightEncoder.getPosition();
  }

  private void setSpark(CANSparkMax spark) {
    spark.restoreFactoryDefaults();
    spark.setIdleMode(IdleMode.kBrake);
    spark.burnFlash();
  }
}

