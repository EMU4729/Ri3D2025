package frc.robot.commands;

import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;

public class ActivateBase extends Command{
    boolean StayUnstowed;

    public ActivateBase(){this(false);}
    public ActivateBase(Boolean StayUnstowed){
        addRequirements(/*Subsystems.coralArm,*/ Subsystems.algaeArm, Subsystems.elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        if(StayUnstowed){return;}
        // Subsystems.coralArm.stow();
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.STOW);
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.STOW);
        super.end(interrupted);
    }
}