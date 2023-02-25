package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class AutoDropHighCone extends CommandBase{
    public final Arm arm;
    public AutoDropHighCone(Arm arm) {
        this.arm = arm;
        arm.RotatePosition(Constants.rotateHighCone);
        arm.elevatorPosition(Constants.elevatorHighCone);
        while(
            !((arm.ElevatorEncoderRight >= (Constants.elevatorHighCone-1)) && (arm.ElevatorEncoderRight <=(Constants.elevatorHighCone+1)))
            &&
            !((arm.RotateEncoder >= (Constants.rotateHighCone-1)) && (arm.RotateEncoder <=(Constants.rotateHighCone+1)))
            ){delay(.1);}
        arm.clampOutPosition();
        while(
            !((arm.ClampEncoderRight >= (Constants.clampRightOut-.5)) && (arm.ClampEncoderRight <=(Constants.clampRightOut+.5)))
            &&
            !((arm.ClampEncoderRight >= (Constants.clampLeftOut-.5)) && (arm.ClampEncoderRight <=(Constants.clampLeftOut+.5)))
            ){delay(.1);}
        arm.RotatePosition(Constants.rotateHybrid);
        arm.elevatorPosition(Constants.elevatorHybrid);






    }
}
