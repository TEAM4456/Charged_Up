package frc.robot.Autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;
//import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoStraight extends CommandBase {
  private Swerve s_Swerve;
//  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public AutoStraight(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    //this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
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
    SmartDashboard.putNumber("Heading at Straight", navRot);
    
    if(Math.abs(navRot) < 20){
    double rotationVal =
        rotationLimiter.calculate(navRot/50);

    /* Drive */
    s_Swerve.drive(
        new Translation2d(0,0).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
    }
  }
}