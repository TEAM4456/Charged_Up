package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class AutoDropLowCone extends CommandBase{
    public final Arm arm;
    public AutoDropLowCone(Arm arm) {
        this.arm = arm;
        arm.RotatePosition(Constants.rotateLowCone);
        arm.elevatorPosition(Constants.elevatorLowCone);
        while(
            !((arm.ElevatorEncoderRight >= (Constants.elevatorLowCone-1)) && (arm.ElevatorEncoderRight <=(Constants.elevatorLowCone+1)))
            &&
            !((arm.RotateEncoder >= (Constants.rotateLowCone-1)) && (arm.RotateEncoder <=(Constants.rotateLowCone+1)))
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

