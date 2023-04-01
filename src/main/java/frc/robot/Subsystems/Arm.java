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

    

    motor17 = new CANSparkMax(17, MotorType.kBrushless); //rotate
    motor17.setOpenLoopRampRate(.5);
    motor17.setClosedLoopRampRate(.1);

    elevatorEncoderRight = motor13.getEncoder();
    elevatorEncoderLeft = motor14.getEncoder();

    

    rotateEncoder = motor17.getEncoder();

    elevatorRightPID = motor13.getPIDController();
    elevatorRightPID.setP(.1);
    elevatorRightPID.setI(0);
    elevatorRightPID.setD(0);
    elevatorRightPID.setFF(0);

    elevatorLeftPID = motor14.getPIDController();
    elevatorLeftPID.setP(.1);
    elevatorLeftPID.setI(0);
    elevatorLeftPID.setD(0);
    elevatorLeftPID.setFF(0);

    

    rotationPID = motor17.getPIDController();
    rotationPID.setP(.04);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setFF(0);
  }

//CLAMP CONTROLS
  

//ELEVATOR CONTROLS

  public void elevatorSpeedIn(){
    motor13.set(.05);
    motor14.set(.05);
  }

  public void elevatorSpeedOut(){
    motor13.set(-.05);
    motor14.set(-.05);
  }

  public void elevatorSpeedStop(){
    motor13.set(0);
    motor14.set(0);
    elevatorPosition(elevatorEncoderRight.getPosition());
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
  public void setHybridPositionRotate(){
    rotatePosition(Constants.armConstants.rotateHybrid);
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
  public void sethighConeRotate(){
    rotatePosition(Constants.armConstants.rotateHighCone);
  }
  public void sethighCubeRotate(){
    rotatePosition(Constants.armConstants.rotateHighCone);
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
  public void setPickupRotatePosition(){
    rotatePosition(Constants.armConstants.rotatePickup);
  }
  public CommandBase bringElevatorInCommand(){
    return run(() -> elevatorPosition(2)).until(() -> Math.abs(elevatorEncoderRight.getPosition() - 2) < 3);
  }
  public CommandBase setCubeHighPositionCommand(){
    return run(() -> setCubeHighPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHighCube) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotateHighCube) < 1);
  }
  public CommandBase setConeHighPositionCommand(){
    return run(() -> setConeHighPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHighCone) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotateHighCone) < 1);
  }
  public CommandBase setCubeLowPositionCommand(){
    return run(() -> setCubeLowPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorLowCube) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotateLowCube) < 1);
  }
  public CommandBase setConeLowPositionCommand(){
    return run(() -> setConeLowPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorLowCone) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotateLowCone) < 1);
  }
  public CommandBase setDrivePositionCommand(){
    return run(() -> setDrivePosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorDrive) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotateDrive) < 1);
  }
  public CommandBase setHybridPositionCommand(){
    return run(() -> setHybridPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHybrid) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotateHybrid) < 1);
  }
  public CommandBase setPickupPositionCommand(){
    return run(() -> setPickupPosition()).until(() -> Math.abs(elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorPickup) < 1 && Math.abs(rotateEncoder.getPosition() - Constants.armConstants.rotatePickup) < 1);
  }
  public void elevatorPosition(double elevatorSetpoint) {
    if(elevatorSetpoint-1>elevatorEncoderRight.getPosition()){
      motor13.set(.3);
      motor14.set(.3);
      System.out.println("go In");
    }
    else if(elevatorSetpoint+1<elevatorEncoderRight.getPosition()){
      motor13.set(-.3);
      motor14.set(-.3);
      System.out.println("go Out");
    }
    else{
      motor13.set(0);
      motor14.set(0);
      elevatorRightPID.setReference(elevatorSetpoint,CANSparkMax.ControlType.kPosition);
      elevatorLeftPID.setReference(elevatorSetpoint,CANSparkMax.ControlType.kPosition);
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

    
    SmartDashboard.putNumber("Rotation Encoder",rotateEncoder.getPosition());

    SmartDashboard.putNumber("Rotation Motor Output",motor17.getAppliedOutput());
    

    SmartDashboard.putNumber("PDP 8", pdp.getCurrent(8));
    SmartDashboard.putNumber("PDP 9", pdp.getCurrent(9));
    SmartDashboard.putNumber("PDP 11", pdp.getCurrent(11));

    SmartDashboard.putNumber("Rotate Temp", motor17.getMotorTemperature());

  }

}