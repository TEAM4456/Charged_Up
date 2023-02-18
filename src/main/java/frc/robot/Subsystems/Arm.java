package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  //elevator
  public final CANSparkMax motor13;
  public final CANSparkMax motor14;

  private RelativeEncoder elevatorEncoderRight;
  private RelativeEncoder elevatorEncoderLeft;


  //clamps
  public final CANSparkMax motor15;
  public final CANSparkMax motor16;

  private RelativeEncoder clampEncoderRight;
  private RelativeEncoder clampEncoderLeft;
  

  //rotate
  public final CANSparkMax motor17;
  private RelativeEncoder rotateEncoder;

  public Arm() {
    motor13 = new CANSparkMax(13, MotorType.kBrushless);
    motor13.setOpenLoopRampRate(.5);
    motor14 = new CANSparkMax(14, MotorType.kBrushless);
    motor14.setOpenLoopRampRate(.5);
    motor14.setInverted(true);

    motor15 = new CANSparkMax(13, MotorType.kBrushless);
    motor15.setOpenLoopRampRate(.5);
    motor16 = new CANSparkMax(14, MotorType.kBrushless);
    motor16.setOpenLoopRampRate(.5);
    motor16.setInverted(true);

    motor17 = new CANSparkMax(17, MotorType.kBrushless);
    motor17.setOpenLoopRampRate(.5);

    elevatorEncoderRight = motor13.getEncoder();
    elevatorEncoderLeft = motor14.getEncoder();

    clampEncoderRight = motor15.getEncoder();
    clampEncoderLeft = motor16.getEncoder();

    rotateEncoder = motor17.getEncoder();
  }
  public void armUp(){
    motor13.set(.2);
    motor14.set(.2);
  }

  public void armDown(){
    motor13.set(-.2);
    motor14.set(-.2);
  }

  public void armStop(){
    motor13.set(0);
    motor14.set(0);
  }

  public void clampInRight(){
    motor15.set(.2);
  }
  public void clampOutRight(){
    motor15.set(-.2);
  }
  public void clampStopRight(){
    motor15.set(0);
  }

  public void clampInLeft(){
    motor16.set(.5);
  }
  public void clampOutLeft(){
    motor16.set(-.5);
  }
  public void clampStopLeft(){
    motor16.set(0);
  }

  public void armRotateUp(){
    motor17.set(.05);
  }
  public void armRotateDown(){
    motor17.set(-.05);
  }
  public void armRotateStop(){
    motor17.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Left",elevatorEncoderLeft.getPosition());
    SmartDashboard.putNumber("Elevator Encoder Right",elevatorEncoderRight.getPosition());

    SmartDashboard.putNumber("Clamp Encoder Left",clampEncoderLeft.getPosition());
    SmartDashboard.putNumber("Clamp Encoder Right",clampEncoderRight.getPosition());

    SmartDashboard.putNumber("Rotation Encoder",rotateEncoder.getPosition());


  }
}