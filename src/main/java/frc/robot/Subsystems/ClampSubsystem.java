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

public class ClampSubsystem extends SubsystemBase {
    //clamps
  public final CANSparkMax motor15;
  public final CANSparkMax motor16;

  public final SparkMaxPIDController clampRightPID;
  public final SparkMaxPIDController clampLeftPID;

  public RelativeEncoder clampEncoderRight;
  public RelativeEncoder clampEncoderLeft;
  public ClampSubsystem(){
    motor15 = new CANSparkMax(15, MotorType.kBrushless); //right clamp
    motor15.setOpenLoopRampRate(.5);

    motor16 = new CANSparkMax(16, MotorType.kBrushless); //left clamp
    motor16.setOpenLoopRampRate(.5);
    motor16.setInverted(true);

    clampEncoderRight = motor15.getEncoder();
    clampEncoderLeft = motor16.getEncoder();

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
  }

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
    if(Math.abs(clampEncoderRight.getPosition() - positionRight) < 3){
      isDone = true;
    }
    return isDone;
  }
  @Override
  public void periodic(){
    SmartDashboard.putNumber("Clamp Left",clampEncoderLeft.getPosition());
    SmartDashboard.putNumber("Clamp Right",clampEncoderRight.getPosition());

    SmartDashboard.putNumber("clamp right motor output", motor15.getAppliedOutput());
    SmartDashboard.putNumber("clamp left motor output", motor16.getAppliedOutput());
  }
}