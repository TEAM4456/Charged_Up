<<<<<<< HEAD
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  //elevator
  public final CANSparkMax motor13;
  public final CANSparkMax motor14;

  public RelativeEncoder elevatorEncoderRight;
  public RelativeEncoder elevatorEncoderLeft;

  public final SparkMaxPIDController elevatorRightPID;
  public final SparkMaxPIDController elevatorLeftPID;


  //clamps
  public final CANSparkMax motor15;
  public final CANSparkMax motor16;

  public final SparkMaxPIDController clampRightPID;
  public final SparkMaxPIDController clampLeftPID;

  public RelativeEncoder clampEncoderRight;
  public RelativeEncoder clampEncoderLeft;
  

  //rotate
  public final CANSparkMax motor17;
  public RelativeEncoder rotateEncoder;
  public final SparkMaxPIDController rotationPID;


  public Arm() {
    motor13 = new CANSparkMax(13, MotorType.kBrushless); //elevator
    motor13.setOpenLoopRampRate(.5);

    motor14 = new CANSparkMax(14, MotorType.kBrushless);
    motor14.setOpenLoopRampRate(.5);
    motor14.setInverted(true);
    //motor14.follow(motor13);

    motor15 = new CANSparkMax(15, MotorType.kBrushless); //right clamp
    motor15.setOpenLoopRampRate(.5);

    motor16 = new CANSparkMax(16, MotorType.kBrushless); //left clamp
    motor16.setOpenLoopRampRate(.5);
    motor16.setInverted(true);

    motor17 = new CANSparkMax(17, MotorType.kBrushless); //rotate
    motor17.setOpenLoopRampRate(.5);
    motor17.setClosedLoopRampRate(.1);

    elevatorEncoderRight = motor13.getEncoder();
    elevatorEncoderLeft = motor14.getEncoder();

    clampEncoderRight = motor15.getEncoder();
    clampEncoderLeft = motor16.getEncoder();

    rotateEncoder = motor17.getEncoder();

    elevatorRightPID = motor13.getPIDController();
    elevatorRightPID.setP(1);
    elevatorRightPID.setI(0);
    elevatorRightPID.setD(0);
    elevatorRightPID.setFF(0);

    elevatorLeftPID = motor14.getPIDController();
    elevatorLeftPID.setP(1);
    elevatorLeftPID.setI(0);
    elevatorLeftPID.setD(0);
    elevatorLeftPID.setFF(0);

    clampRightPID = motor15.getPIDController();
    clampRightPID.setP(.75);
    clampRightPID.setI(0);
    clampRightPID.setD(0);
    clampRightPID.setFF(0);

    clampLeftPID = motor16.getPIDController();
    clampLeftPID.setP(.75);
    clampLeftPID.setI(0);
    clampLeftPID.setD(0);
    clampLeftPID.setFF(0);

    rotationPID = motor17.getPIDController();
    rotationPID.setP(.04);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setFF(0);
  }
  /* old - mark for deletion 
  public void armUp(){
    elevatorRightPID.setReference(elevatorEncoderRight.getPosition() + 4,CANSparkMax.ControlType.kPosition); 
  //  elevatorLeftPID.setReference(elevatorEncoderRight.getPosition() + 4,CANSparkMax.ControlType.kPosition); 
  }

  public void armDown(){
    elevatorRightPID.setReference(elevatorEncoderRight.getPosition() - 2,CANSparkMax.ControlType.kPosition); 
  //  elevatorLeftPID.setReference(elevatorEncoderRight.getPosition() - 2,CANSparkMax.ControlType.kPosition); 
  }
  */

//CLAMP CONTROLS
  public void clampInPositionCone(){
    clampRightPID.setReference(Constants.armConstants.clampRightPickupCone,CANSparkMax.ControlType.kPosition); 
    clampLeftPID.setReference(Constants.armConstants.clampLeftPickupCone,CANSparkMax.ControlType.kPosition);  
  }
  public void clampInPositionCube(){
    clampRightPID.setReference(Constants.armConstants.clampRightPickupCube,CANSparkMax.ControlType.kPosition); 
    clampLeftPID.setReference(Constants.armConstants.clampLeftPickupCube,CANSparkMax.ControlType.kPosition);  
  }
  
  public void clampOutPosition(){
    clampRightPID.setReference(Constants.armConstants.clampRightDrop,CANSparkMax.ControlType.kPosition);
    clampLeftPID.setReference(Constants.armConstants.clampLeftDrop,CANSparkMax.ControlType.kPosition);  
  }

  public void clampInRight(){
    //double setpoint = (clampEncoderRight.getPosition() + 0.5);
    clampRightPID.setReference(clampEncoderRight.getPosition() + 0.25,CANSparkMax.ControlType.kPosition); 
  }
  public void clampOutRight(){
    //double setpoint = (clampEncoderRight.getPosition() - 0.5);
    clampRightPID.setReference(clampEncoderRight.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
  }

  public void clampInLeft(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() + 0.25,CANSparkMax.ControlType.kPosition);
  }
  
  public void clampOutLeft(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
  }
  public void clampOut(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
    clampRightPID.setReference(clampEncoderRight.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
  }
  public void clampIn(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() + 0.25,CANSparkMax.ControlType.kPosition);
    clampRightPID.setReference(clampEncoderRight.getPosition() + 0.25,CANSparkMax.ControlType.kPosition);
  }
  public void clampReset(){
    clampLeftPID.setReference(0,CANSparkMax.ControlType.kPosition);
    clampRightPID.setReference(0,CANSparkMax.ControlType.kPosition);
  }

//ELEVATOR CONTROLS
/* 
  public void elevatorOut(){
    double elevatorSetPoint = (elevatorEncoderRight.getPosition() - 2);
    elevatorRightPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
    elevatorLeftPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
    //elevatorLeftPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
  }
  public void elevatorIn(){
    double elevatorSetPoint = (elevatorEncoderRight.getPosition() + 2);
    elevatorRightPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
    elevatorLeftPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
    //elevatorLeftPID.setReference(elevatorSetPoint,CANSparkMax.ControlType.kPosition);
  }
*/
  public void elevatorSpeedIn(){
    motor13.set(.2);
    motor14.set(.2);
  }

  public void elevatorSpeedOut(){
    motor13.set(-.2);
    motor14.set(-.2);
  }

  public void elevatorSpeedStop(){
    motor13.set(0);
    motor14.set(0);
  }
  public void elevatorPosition(double elevatorSetpoint) {
    if(elevatorSetpoint>(elevatorEncoderRight.getPosition())){
      motor13.set(.5);
      motor14.set(.5);
    }
    else{
      motor13.set(-.5);
      motor14.set(-.5);
    }
    while(
      !(elevatorSetpoint+3>elevatorEncoderRight.getPosition()) 
      || 
      !(elevatorEncoderRight.getPosition()>elevatorSetpoint-3))
      {
        Timer.delay(.05);
      }
    motor13.set(0);
    motor14.set(0);
    elevatorRightPID.setReference(elevatorSetpoint,CANSparkMax.ControlType.kPosition);
  }


//ROTATE CONTROLS
  public void rotateSpeedUp(){
    motor17.set(-1);
  }

  public void rotateSpeedDown(){
    motor17.set(1);
  }

  public void rotateSpeedStop(){
    motor17.set(0);
  }

  public void rotateSpeedHold(){
    rotationPID.setReference(rotateEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  
  public void rotatePosition(double rotateSetpoint) {
    rotationPID.setReference(rotateSetpoint,CANSparkMax.ControlType.kPosition);
  }

/* ROTATION CONTROL BY POSITION 
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
 */
  

  
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
    SmartDashboard.putNumber("PDP 11", pdp.getCurrent(11));

    SmartDashboard.putNumber("Rotate Temp", motor17.getMotorTemperature());

  }
=======
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  
  //elevator
  public final CANSparkMax motor13;
  public final CANSparkMax motor14;

  public RelativeEncoder elevatorEncoderRight;
  public RelativeEncoder elevatorEncoderLeft;

  public final SparkMaxPIDController elevatorRightPID;
  public final SparkMaxPIDController elevatorLeftPID;


  //clamps
  public final CANSparkMax motor15;
  public final CANSparkMax motor16;

  public final SparkMaxPIDController clampRightPID;
  public final SparkMaxPIDController clampLeftPID;

  public RelativeEncoder clampEncoderRight;
  public RelativeEncoder clampEncoderLeft;
  

  //rotate
  public final CANSparkMax motor17;
  public RelativeEncoder rotateEncoder;
  public final SparkMaxPIDController rotationPID;


  public Arm() {
    motor13 = new CANSparkMax(13, MotorType.kBrushless); //elevator
    motor13.setOpenLoopRampRate(.5);

    motor14 = new CANSparkMax(14, MotorType.kBrushless);
    motor14.setOpenLoopRampRate(.5);
    motor14.setInverted(true);
    //motor14.follow(motor13);

    motor15 = new CANSparkMax(15, MotorType.kBrushless); //right clamp
    motor15.setOpenLoopRampRate(.5);

    motor16 = new CANSparkMax(16, MotorType.kBrushless); //left clamp
    motor16.setOpenLoopRampRate(.5);
    motor16.setInverted(true);

    motor17 = new CANSparkMax(17, MotorType.kBrushless); //rotate
    motor17.setOpenLoopRampRate(.5);
    motor17.setClosedLoopRampRate(.1);

    elevatorEncoderRight = motor13.getEncoder();
    elevatorEncoderLeft = motor14.getEncoder();

    clampEncoderRight = motor15.getEncoder();
    clampEncoderLeft = motor16.getEncoder();

    rotateEncoder = motor17.getEncoder();

    elevatorRightPID = motor13.getPIDController();
    elevatorRightPID.setP(1);
    elevatorRightPID.setI(0);
    elevatorRightPID.setD(0);
    elevatorRightPID.setFF(0);

    elevatorLeftPID = motor14.getPIDController();
    elevatorLeftPID.setP(1);
    elevatorLeftPID.setI(0);
    elevatorLeftPID.setD(0);
    elevatorLeftPID.setFF(0);

    clampRightPID = motor15.getPIDController();
    clampRightPID.setP(1);
    clampRightPID.setI(0);
    clampRightPID.setD(0);
    clampRightPID.setFF(0);

    clampLeftPID = motor16.getPIDController();
    clampLeftPID.setP(1);
    clampLeftPID.setI(0);
    clampLeftPID.setD(0);
    clampLeftPID.setFF(0);

    rotationPID = motor17.getPIDController();
    rotationPID.setP(.04);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setFF(0);
  }

//CLAMP CONTROLS
  public void clampInPositionCone(){
    clampRightPID.setReference(Constants.armConstants.clampRightPickupCone,CANSparkMax.ControlType.kPosition); 
    clampLeftPID.setReference(Constants.armConstants.clampLeftPickupCone,CANSparkMax.ControlType.kPosition);  
  }
  public void clampInPositionCube(){
    clampRightPID.setReference(Constants.armConstants.clampRightPickupCube,CANSparkMax.ControlType.kPosition); 
    clampLeftPID.setReference(Constants.armConstants.clampLeftPickupCube,CANSparkMax.ControlType.kPosition);  
  }
  
  public void clampOutPosition(){
    clampRightPID.setReference(Constants.armConstants.clampRightDrop,CANSparkMax.ControlType.kPosition);
    clampLeftPID.setReference(Constants.armConstants.clampLeftDrop,CANSparkMax.ControlType.kPosition);  
  }

  public void clampInRight(){
    //double setpoint = (clampEncoderRight.getPosition() + 0.5);
    clampRightPID.setReference(clampEncoderRight.getPosition() + 0.25,CANSparkMax.ControlType.kPosition); 
  }
  public void clampOutRight(){
    //double setpoint = (clampEncoderRight.getPosition() - 0.5);
    clampRightPID.setReference(clampEncoderRight.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
  }

  public void clampInLeft(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() + 0.25,CANSparkMax.ControlType.kPosition);
  }
  
  public void clampOutLeft(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
  }
  public void clampOut(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
    clampRightPID.setReference(clampEncoderRight.getPosition() - 0.25,CANSparkMax.ControlType.kPosition);
  }
  public void clampIn(){
    clampLeftPID.setReference(clampEncoderLeft.getPosition() + 0.25,CANSparkMax.ControlType.kPosition);
    clampRightPID.setReference(clampEncoderRight.getPosition() + 0.25,CANSparkMax.ControlType.kPosition);
  }
  public void clampReset(){
    clampLeftPID.setReference(.25,CANSparkMax.ControlType.kPosition);
    clampRightPID.setReference(.25,CANSparkMax.ControlType.kPosition);
  }
  public boolean nearTarget(double positionRight){
    boolean isDone = false;
    if(Math.abs(clampEncoderRight.getPosition() - positionRight) < .25){
      isDone = true;
    }
    return isDone;
  }

//ELEVATOR CONTROLS

  public void elevatorSpeedIn(){
    motor13.set(.2);
    motor14.set(.2);
  }

  public void elevatorSpeedOut(){
    motor13.set(-.2);
    motor14.set(-.2);
  }

  public void elevatorSpeedStop(){
    motor13.set(0);
    motor14.set(0);
  }
  /* 
  public void elevatorPosition(double elevatorSetpoint) {
    double speed = 0;
    if(elevatorSetpoint>(elevatorEncoderRight.getPosition())){
      speed = .5;
    }
    else{
      speed = -.5;
    }
    motor13.set(speed);
    motor14.set(speed);
    while((Math.abs(elevatorSetpoint-elevatorEncoderRight.getPosition()))< 5){
      motor13.set(speed/2);
      motor14.set(speed/2);
    }
    while((Math.abs(elevatorSetpoint-elevatorEncoderRight.getPosition()))< 3){
      {
        Timer.delay(.01);
      }
    }
    motor13.set(0);
    motor14.set(0);
    elevatorRightPID.setReference(elevatorSetpoint,CANSparkMax.ControlType.kPosition);
  }
  */
  public void setDrivePosition(){
    elevatorPosition(Constants.armConstants.elevatorDrive);
    rotatePosition(Constants.armConstants.rotateDrive);
  }
  public void setHybridPosition(){
    elevatorPosition(Constants.armConstants.elevatorHybrid);
    rotatePosition(Constants.armConstants.rotateHybrid);
  }
  public void setConeLowPosition(){
    elevatorPosition(Constants.armConstants.elevatorLowCone);
    rotatePosition(Constants.armConstants.rotateLowCone);
  }
  public void setConeHighPosition(){
    rotatePosition(Constants.armConstants.rotateHighCone);
    elevatorPosition(Constants.armConstants.elevatorHighCone);
    
  }
  public void setCubeHighPosition(){
    rotatePosition(Constants.armConstants.rotateHighCube);
    elevatorPosition(Constants.armConstants.elevatorHighCube);
  }
  public void setCubeLowPosition(){
    elevatorPosition(Constants.armConstants.elevatorLowCube);
    rotatePosition(Constants.armConstants.rotateLowCube);
  }
  public void setPickupPosition(){
    elevatorPosition(Constants.armConstants.elevatorPickup);
    rotatePosition(Constants.armConstants.rotatePickup);
  }
  public CommandBase setCubeHighPositionCommand(){
    return run(() -> setCubeHighPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHighCube) < 1);
  }
  public CommandBase setConeHighPositionCommand(){
    return run(() -> setConeHighPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHighCone) < 1);
  }
  public CommandBase setCubeLowPositionCommand(){
    return run(() -> setCubeLowPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorLowCube) < 1);
  }
  public CommandBase setConeLowPositionCommand(){
    return run(() -> setConeLowPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorLowCone) < 1);
  }
  public CommandBase setDrivePositionCommand(){
    return run(() -> setDrivePosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorDrive) < 1);
  }
  public CommandBase setHybridPositionCommand(){
    return run(() -> setHybridPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHybrid) < 1);
  }
  public CommandBase setPickupPositionCommand(){
    return run(() -> setPickupPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorPickup) < 1);
  }
  public void elevatorPosition(double elevatorSetpoint) {
    if(elevatorSetpoint-3>elevatorEncoderRight.getPosition()){
      motor13.set(.75);
      motor14.set(.75);
      System.out.println("go In");
    }
    else if(elevatorSetpoint+3<elevatorEncoderRight.getPosition()){
      motor13.set(-.75);
      motor14.set(-.75);
      System.out.println("go Out");
    }
    else{
      motor13.set(0);
      motor14.set(0);
      elevatorRightPID.setReference(elevatorSetpoint,CANSparkMax.ControlType.kPosition);
      System.out.println("setReferece");
    }
    
  }
  public double getElevatorEncoder(){
    return elevatorEncoderRight.getPosition();
  }


//ROTATE CONTROLS
  public void rotateSpeedUp(){
    motor17.set(-1);
  }

  public void rotateSpeedDown(){
    motor17.set(1);
  }

  public void rotateSpeedStop(){
    motor17.set(0);
  }

  public void rotateSpeedHold(){
    rotationPID.setReference(rotateEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  
  public void rotatePosition(double rotateSetpoint) {
    rotationPID.setReference(rotateSetpoint,CANSparkMax.ControlType.kPosition);
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
    SmartDashboard.putNumber("PDP 11", pdp.getCurrent(11));

    SmartDashboard.putNumber("Rotate Temp", motor17.getMotorTemperature());

  }

>>>>>>> master-temp
}