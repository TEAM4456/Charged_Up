// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public LimeLight() {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //NetworkTableEntry tx = table.getEntry("tx");
  //NetworkTableEntry ty = table.getEntry("ty");
  //NetworkTableEntry ta = table.getEntry("ta");
  
  //double x = tx.getDouble(0.0);//x offset
  //double y = ty.getDouble(0.0);//y offset
  //double area = ta.getDouble(0.0);//area
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
    SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
    SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
  }

  
}