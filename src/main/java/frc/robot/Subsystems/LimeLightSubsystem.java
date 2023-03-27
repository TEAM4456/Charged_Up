// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
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
  public PhotonCamera camera = new PhotonCamera("OV5647"); // camera name which is in UI
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
    if(aquired){
    double moveX = aprilTagValues.getX();
    double moveY = aprilTagValues.getY();
    SmartDashboard.putNumber("apriltag X", moveX);
    SmartDashboard.putNumber("apriltag Y", moveY);
    moveX -= 1.25;
    if(moveX > .25){
      moveX = .25;
    }
    else if(moveX < -.25){
      moveX = -.25;
    }
    moveY += .5;
    if(moveY > .15){
      moveY = .15;
    }
    else if(moveY < -.15){
      moveY = -.15;
    }

    
    /* Get Values, Deadband*/
    /* Drive */
    s_Swerve.drive(
        new Translation2d(moveX,moveY).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
  }
  }
  public CommandBase autoPickupCommand() {
    SmartDashboard.putBoolean("command ran", true);
    return run(() -> autoPickUp()).until(() -> (Math.abs(pitch)<1));
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
      if(targetNum == 4){
        aprilTagValues = target.getBestCameraToTarget();
      }
      
    }
    SmartDashboard.putBoolean("Apriltag Aquired", aquired);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch",pitch);
    SmartDashboard.putNumber("Target",targetNum);
    
  }
}
