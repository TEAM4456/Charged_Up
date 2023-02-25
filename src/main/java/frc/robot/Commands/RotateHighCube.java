package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class RotateHighCube extends CommandBase{
    public final Arm arm;
    public RotateHighCube(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(100);
    }

}
