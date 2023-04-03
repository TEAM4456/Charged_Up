package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ClampSubsystem;

public class ClampPositionCone extends CommandBase{
    public final ClampSubsystem c;
    public ClampPositionCone(ClampSubsystem c){
        this.c = c;
    }
    public void execute() {
        c.clampRightPID.setP(2);
        c.clampLeftPID.setP(2);
        c.clampInPositionCone();
    }
    public boolean isFinished() {
        return c.nearTarget(Constants.armConstants.clampRightPickupCube);
      }
}
