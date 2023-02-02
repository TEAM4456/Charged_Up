// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;
//import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class limeLighSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier strafeSup;
//  private BooleanSupplier robotCentricSup;
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);

  public limeLighSwerve(
      Swerve s_Swerve,
      DoubleSupplier strafeSup
      /*BooleanSupplier robotCentricSup*/) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    
    this.strafeSup = strafeSup;

    //this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    
    /* Drive */
    s_Swerve.drive(
        new Translation2d(0, strafeVal).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
  }
}
