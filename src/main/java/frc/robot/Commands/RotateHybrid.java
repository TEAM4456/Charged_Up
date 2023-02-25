package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class RotateHybrid extends CommandBase{
    public final Arm arm;
    public RotateHybrid(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(100);
    }

}
