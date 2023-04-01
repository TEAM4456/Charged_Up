package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ClampSubsystem;

public class ClampPositionDrop extends CommandBase{
    public final ClampSubsystem c;
    public ClampPositionDrop(ClampSubsystem c){
      this.c = c;
  }
    public void execute() {
        c.clampLeftPID.setP(.5);
        c.clampRightPID.setP(.5);
        c.clampOutPosition();
    }

  @Override
  public boolean isFinished() {
    return c.nearTarget(Constants.armConstants.clampRightDrop);
  }
}
