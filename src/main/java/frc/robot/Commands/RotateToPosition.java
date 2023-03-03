package frc.robot.Commands;
import frc.robot.Constants;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class RotateToPosition extends CommandBase{
    public final Arm arm;
    public RotateToPosition(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(100);
    }
    //public void end(boolean interrupted){
    //    arm.clampOutPosition();
    //}
}