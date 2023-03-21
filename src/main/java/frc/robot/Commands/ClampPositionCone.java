package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampPositionCone extends CommandBase{
    public final Arm arm;
    public ClampPositionCone(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampRightPID.setP(4);
        arm.clampLeftPID.setP(4);
        arm.clampInPositionCone();
    }
}
