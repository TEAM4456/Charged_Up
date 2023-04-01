package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ClampSubsystem;

public class ClampPositionCube extends CommandBase{
    public final ClampSubsystem c;
    public ClampPositionCube(ClampSubsystem c){
        this.c = c;
    }
    public void execute() {
        c.clampRightPID.setP(.5);
        c.clampLeftPID.setP(.5);
        c.clampInPositionCube();
    }
    public boolean isFinished() {
        return c.nearTarget(Constants.armConstants.clampRightPickupCube);
      }
}
