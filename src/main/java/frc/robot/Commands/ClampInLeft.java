package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ClampSubsystem;

public class ClampInLeft extends CommandBase{
    public final ClampSubsystem c;
    public ClampInLeft(ClampSubsystem c){
        this.c = c;
    }
    public void execute() {
        
        c.clampInLeft();
    }
}
