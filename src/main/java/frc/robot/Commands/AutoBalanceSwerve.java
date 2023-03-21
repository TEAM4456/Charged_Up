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

public class AutoBalanceSwerve extends CommandBase {
  public Swerve s_Swerve;
//  private BooleanSupplier robotCentricSup;
  public SlewRateLimiter translationLimiter = new SlewRateLimiter(1.5);
  public AutoBalanceSwerve(
      Swerve s_Swerve
      /*BooleanSupplier robotCentricSup*/) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    //this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    double translationVal = 0;
    double roll = s_Swerve.getGyroRoll();
    if(Math.abs(roll)>10){
     translationVal = translationLimiter.calculate(-roll/115);
      SmartDashboard.putNumber("AutoBalance in Execute", roll);
    }
    
    /* Get Values, Deadband*/
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal,0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
  }
}
