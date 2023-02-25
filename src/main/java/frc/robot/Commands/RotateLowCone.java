package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class RotateLowCone extends CommandBase{
    public final Arm arm;
    public RotateLowCone(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(100);
    }

}
