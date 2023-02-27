
package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ElevatorOut extends CommandBase{
    public final Arm arm;
    public ElevatorOut(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        if(arm.elevatorEncoderRight.getPosition()>-103){
          arm.elevatorSpeedOut();
        }
        else{
            arm.elevatorSpeedStop();
        }
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        arm.motor17.set(0);
        arm.elevatorSpeedStop();
      }
}