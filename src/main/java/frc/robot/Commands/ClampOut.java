package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampOut extends CommandBase{
    public final Arm arm;
    public ClampOut(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampOut();
    }
}