package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class RotateToPosition extends CommandBase{
    public final Arm arm;
    public RotateToPosition(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.armRotateHigh();
    }
    //public void end(boolean interrupted){
    //    arm.clampOutPosition();
    //}
}