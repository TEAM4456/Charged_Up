// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class LimeLightSubsystem extends SubsystemBase {
  
  /** Creates a new ExampleSubsystem. */
  public boolean aquired;
  public double yaw;
  public double pitch;
  public double area;
  public double targetNum;
  public Transform3d aprilTagValues;
  double xRel;
  double yRel;
  public final PhotonCamera camera = new PhotonCamera("OV5647"); // camera name which is in UI
  public final Swerve s_Swerve;
  public LimeLightSubsystem(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    aquired = false;
    yaw = 0;
    pitch = 0;
    area = 0;
    targetNum = 0;
   
  }
  public void autoPickUp(){
      SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
      double navRot = s_Swerve.getHeading();
   
    if(navRot > 20){
      navRot = 10;
    }
    else if(navRot < -20){
      navRot = -10;
    }
    else if(navRot < 20 && navRot > 2.5){
      navRot = 2.5;
    }
    else if(navRot > -20 && navRot < -2.5){
      navRot = -2.5;
    }
    if(Math.abs(navRot) < .25){
      navRot = 0;
    }
    SmartDashboard.putNumber("Heading at Straightv2", navRot);
    
    double rotationVal =
        rotationLimiter.calculate(navRot/50);
    if(aquired){
      double moveX = xRel;
      double moveY = yRel;
      SmartDashboard.putNumber("apriltag X", moveX);
      SmartDashboard.putNumber("apriltag Y", moveY);
      moveX -= 1.3;
      if(moveX > .25){
        moveX = .25;
      }
      else if(moveX < -.25){
        moveX = -.25;
      }
      moveY += .55;
      if(moveY > .15){
        moveY = .15;
      }
      else if(moveY < -.15){
        moveY = -.15;
      }

      
      /* Get Values, Deadband*/
      /* Drive */
      if(Math.abs(moveX) < .05){
        moveX = 0;
      }
      if(Math.abs(moveY) < .05){
        moveY = 0;
      }
      s_Swerve.drive(
          new Translation2d(moveX,moveY).times(Constants.Swerve.maxSpeed),
          rotationVal * Constants.Swerve.maxAngularVelocity,
          //!robotCentricSup.getAsBoolean(),
          true);
    }
    else{
      s_Swerve.drive(
        new Translation2d(0,0).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
    }
  }
  public void autoConeLineup(){
  if(aquired){
    double moveX = xRel;
    double moveY = yRel;
    SmartDashboard.putNumber("apriltag X", moveX);
    SmartDashboard.putNumber("apriltag Y", moveY);
    moveX -= .87;
    if(moveX > .25){
      moveX = .25;
    }
    else if(moveX < -.25){
      moveX = -.25;
    }
    moveY += .34;
    if(moveY > .15){
      moveY = .15;
    }
    else if(moveY < -.15){
      moveY = -.15;
    }

    
    /* Get Values, Deadband*/
    /* Drive */
    if(Math.abs(moveX) < .05){
      moveX = 0;
    }
    if(Math.abs(moveY) < .05){
      moveY = 0;
    }
    s_Swerve.drive(
        new Translation2d(-moveX/1.5,-moveY/1.5).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
    
  }
  else{
    s_Swerve.drive(
      new Translation2d(0,0).times(Constants.Swerve.maxSpeed),
      0 * Constants.Swerve.maxAngularVelocity,
      //!robotCentricSup.getAsBoolean(),
      true);
  }
}
  public CommandBase autoPickupCommand() {
    return run(() -> autoPickUp()).until(() -> ((Math.abs(xRel-1.3)< .05) && Math.abs(yRel+.55) < 0.05));
  }
  public CommandBase autoPickupCommandGeneral() {
    return run(() -> autoPickUp()).until(() -> ((Math.abs(xRel-1.3)< .25) && Math.abs(yRel+.55) < 0.05));
  }
  public CommandBase autoConeLineupCommand() {
    return run(() -> autoConeLineup()).until(() -> ((Math.abs(xRel-.87)< .05) && Math.abs(yRel+.34) < .05));
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    var result = camera.getLatestResult();
    if(result.hasTargets()){
      aquired = true;
    }
    else{
      aquired = false;
    }
    if(aquired){
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      targetNum = target.getFiducialId();
      if(targetNum > 0){
        aprilTagValues = target.getBestCameraToTarget();
        xRel = aprilTagValues.getX();
        yRel = aprilTagValues.getY();
      }
      
    }
    SmartDashboard.putBoolean("Apriltag Aquired", aquired);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch",pitch);
    SmartDashboard.putNumber("Target",targetNum);
    
  }
}
