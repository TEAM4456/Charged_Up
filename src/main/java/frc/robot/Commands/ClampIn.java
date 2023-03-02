package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampIn extends CommandBase{
    public final Arm arm;
    public ClampIn(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampIn();
    }
}