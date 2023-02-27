package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampPositionDrop extends CommandBase{
    public final Arm arm;
    public ClampPositionDrop(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampLeftPID.setP(.75);
        arm.clampRightPID.setP(.75);
        arm.clampOutPosition();
    }
}
