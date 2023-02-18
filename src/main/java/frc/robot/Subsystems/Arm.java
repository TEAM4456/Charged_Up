package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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

  private final SparkMaxPIDController elevatorRightPID;
  private final SparkMaxPIDController elevatorLeftPID;


  //clamps
  public final CANSparkMax motor15;
  public final CANSparkMax motor16;

  private final SparkMaxPIDController clampRightPID;
  private final SparkMaxPIDController clampLeftPID;

  private RelativeEncoder clampEncoderRight;
  private RelativeEncoder clampEncoderLeft;
  

  //rotate
  public final CANSparkMax motor17;
  private RelativeEncoder rotateEncoder;
  private final SparkMaxPIDController rotationPID;


  public Arm() {
    motor13 = new CANSparkMax(13, MotorType.kBrushless);
    motor13.setOpenLoopRampRate(.5);

    motor14 = new CANSparkMax(14, MotorType.kBrushless);
    motor14.setOpenLoopRampRate(.5);
    motor14.setInverted(true);

    motor15 = new CANSparkMax(15, MotorType.kBrushless);
    motor15.setOpenLoopRampRate(.5);

    motor16 = new CANSparkMax(16, MotorType.kBrushless);
    motor16.setOpenLoopRampRate(.5);
    motor16.setInverted(true);

    motor17 = new CANSparkMax(17, MotorType.kBrushless);
    motor17.setOpenLoopRampRate(.5);

    elevatorEncoderRight = motor13.getEncoder();
    elevatorEncoderLeft = motor14.getEncoder();

    clampEncoderRight = motor15.getEncoder();
    clampEncoderLeft = motor16.getEncoder();

    rotateEncoder = motor17.getEncoder();

    elevatorRightPID = motor13.getPIDController();
    elevatorRightPID.setP(0);
    elevatorRightPID.setI(0);
    elevatorRightPID.setD(0);
    elevatorRightPID.setFF(0);

    elevatorLeftPID = motor14.getPIDController();
    elevatorLeftPID.setP(0);
    elevatorLeftPID.setI(0);
    elevatorLeftPID.setD(0);
    elevatorLeftPID.setFF(0);

    clampRightPID = motor15.getPIDController();
    clampRightPID.setP(0);
    clampRightPID.setI(0);
    clampRightPID.setD(0);
    clampRightPID.setFF(0);

    clampLeftPID = motor16.getPIDController();
    clampLeftPID.setP(0);
    clampLeftPID.setI(0);
    clampLeftPID.setD(0);
    clampLeftPID.setFF(0);

    rotationPID = motor17.getPIDController();
    rotationPID.setP(0);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setFF(0);
  }
  public void armUp(){
    elevatorEncoderRight.setPosition(elevatorEncoderRight.getPosition() + 1);
    elevatorEncoderLeft.setPosition(elevatorEncoderLeft.getPosition() + 1);
  }

  public void armDown(){
    elevatorEncoderRight.setPosition(elevatorEncoderRight.getPosition() - 1);
    elevatorEncoderLeft.setPosition(elevatorEncoderLeft.getPosition() - 1);
  }

  public void clampInRight(){
    clampEncoderRight.setPosition(clampEncoderRight.getPosition() - 1);
  }
  public void clampOutRight(){
    clampEncoderRight.setPosition(clampEncoderRight.getPosition() + 1);
  }

  public void clampInLeft(){
    clampEncoderLeft.setPosition(clampEncoderLeft.getPosition() - 1);
  }
  public void clampOutLeft(){
    clampEncoderLeft.setPosition(clampEncoderLeft.getPosition() + 1);
  }

  public void armRotateUp(){
    rotateEncoder.setPosition(rotateEncoder.getPosition() + 1);
  }
  public void armRotateDown(){
    rotateEncoder.setPosition(rotateEncoder.getPosition() - 1);
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