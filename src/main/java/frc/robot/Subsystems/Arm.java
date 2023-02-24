package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
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
    motor17.setClosedLoopRampRate(.1);

    elevatorEncoderRight = motor13.getEncoder();
    elevatorEncoderLeft = motor14.getEncoder();

    clampEncoderRight = motor15.getEncoder();
    clampEncoderLeft = motor16.getEncoder();

    rotateEncoder = motor17.getEncoder();

    elevatorRightPID = motor13.getPIDController();
    elevatorRightPID.setP(.5);
    elevatorRightPID.setI(0);
    elevatorRightPID.setD(0);
    elevatorRightPID.setFF(0);

    elevatorLeftPID = motor14.getPIDController();
    elevatorLeftPID.setP(.5);
    elevatorLeftPID.setI(0);
    elevatorLeftPID.setD(0);
    elevatorLeftPID.setFF(0);

    clampRightPID = motor15.getPIDController();
    clampRightPID.setP(.1);
    clampRightPID.setI(0);
    clampRightPID.setD(0);
    clampRightPID.setFF(0);

    clampLeftPID = motor16.getPIDController();
    clampLeftPID.setP(.1);
    clampLeftPID.setI(0);
    clampLeftPID.setD(0);
    clampLeftPID.setFF(0);

    rotationPID = motor17.getPIDController();
    rotationPID.setP(.04);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setFF(0);
  }
  public void armUp(){
    elevatorRightPID.setReference(elevatorEncoderRight.getPosition() + 4,CANSparkMax.ControlType.kPosition); 
    elevatorLeftPID.setReference(elevatorEncoderRight.getPosition() + 4,CANSparkMax.ControlType.kPosition); 
  }

  public void armDown(){
    elevatorRightPID.setReference(elevatorEncoderRight.getPosition() - 2,CANSparkMax.ControlType.kPosition); 
    elevatorLeftPID.setReference(elevatorEncoderRight.getPosition() - 2,CANSparkMax.ControlType.kPosition); 
  }

  public void clampInPosition(){
    clampRightPID.setReference(-3.8,CANSparkMax.ControlType.kPosition); 
    clampLeftPID.setReference(12,CANSparkMax.ControlType.kPosition);  
  }
  public void clampOutPosition(){
    clampRightPID.setReference(-7,CANSparkMax.ControlType.kPosition);
    clampLeftPID.setReference(9,CANSparkMax.ControlType.kPosition);  
  }

  public void clampInRight(){
    //double setpoint = (clampEncoderRight.getPosition() + 0.5);
    clampRightPID.setReference(clampEncoderRight.getPosition() + 0.5,CANSparkMax.ControlType.kPosition); 
  }
  public void clampOutRight(){
    //double setpoint = (clampEncoderRight.getPosition() - 0.5);
    clampRightPID.setReference(clampEncoderRight.getPosition() - 0.5,CANSparkMax.ControlType.kPosition);
  }

  public void clampInLeft(){
    clampLeftPID.setReference(clampEncoderRight.getPosition() + 0.5,CANSparkMax.ControlType.kPosition);
  }
  
  public void clampOutLeft(){
    clampLeftPID.setReference(clampEncoderRight.getPosition() - 0.5,CANSparkMax.ControlType.kPosition);
  }

  public void armRotateUp(){
    double rotateSetpoint = (rotateEncoder.getPosition() - 2);
    rotationPID.setReference(rotateSetpoint,CANSparkMax.ControlType.kPosition);
  }
  public void armRotateDown(){
    double rotateSetpoint = (rotateEncoder.getPosition() + 2);
    rotationPID.setReference(rotateSetpoint,CANSparkMax.ControlType.kPosition);
  }
  public void armRotateHigh(){
    rotationPID.setReference(-80,CANSparkMax.ControlType.kPosition);
 }

  public void elevatorOut(){
    double elevatorSetPoint = (elevatorEncoderRight.getPosition() - 2);
    elevatorRightPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
    elevatorLeftPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
  }
  public void elevatorIn(){
    double elevatorSetPoint = (elevatorEncoderRight.getPosition() + 2);
    elevatorRightPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
    elevatorLeftPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
  }

  public void rotateSpeedUp(){
    motor17.set(1);
  }

  public void rotateSpeedDown(){
    motor17.set(-1);
  }

  public void rotateSpeedHold(){
    rotationPID.setReference(rotateEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public void rotateSpeedStop(){
    motor17.set(0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Left",elevatorEncoderLeft.getPosition());
    SmartDashboard.putNumber("Elevator Right",elevatorEncoderRight.getPosition());

    SmartDashboard.putNumber("Clamp Left",clampEncoderLeft.getPosition());
    SmartDashboard.putNumber("Clamp Right",clampEncoderRight.getPosition());

    SmartDashboard.putNumber("Rotation Encoder",rotateEncoder.getPosition());

    SmartDashboard.putNumber("Rotation Motor Output",motor17.getAppliedOutput());
    SmartDashboard.putNumber("clamp right motor output", motor15.getAppliedOutput());
    SmartDashboard.putNumber("clamp left motor output", motor16.getAppliedOutput());

    SmartDashboard.putNumber("PDP 8", pdp.getCurrent(8));
    SmartDashboard.putNumber("PDP 9", pdp.getCurrent(9));

    SmartDashboard.putNumber("Rotate Status", motor17.getMotorTemperature());

  }
}