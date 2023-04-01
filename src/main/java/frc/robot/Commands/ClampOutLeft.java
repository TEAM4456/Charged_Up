package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ClampSubsystem;

public class ClampOutLeft extends CommandBase{
    public final ClampSubsystem c;
    public ClampOutLeft(ClampSubsystem c){
        this.c = c;
    }
    public void execute() {
        
        c.clampOutLeft();
    }
}
