// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.Swerve;
//import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class limeLightSwerve extends CommandBase {
  public Swerve s_Swerve;
//  private BooleanSupplier robotCentricSup;
  public SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  public double xMove;
  public double yMove;
  public double aquired;
  public final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public final NetworkTableEntry tx = table.getEntry("tx");
  public final NetworkTableEntry ty = table.getEntry("ty");
  public final NetworkTableEntry tv = table.getEntry("tv");
  public limeLightSwerve(
      Swerve s_Swerve
      /*BooleanSupplier robotCentricSup*/) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    xMove = 0;
    //this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    double strafeVal = 0;
    xMove = tx.getDouble(0.0);
    aquired = tv.getDouble(0.0);
    if((xMove>.5 || xMove <-.5) &&(aquired > .5)){
      strafeVal =strafeLimiter.calculate(-xMove/150);
      SmartDashboard.putNumber("Aquire in execute", aquired);
      SmartDashboard.putNumber("LimelightX in execute", xMove);
    }
    else{
      strafeVal = 0;
    }
    
    /* Get Values, Deadband*/
    /* Drive */
    s_Swerve.drive(
        new Translation2d(0, strafeVal).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
  }
}
