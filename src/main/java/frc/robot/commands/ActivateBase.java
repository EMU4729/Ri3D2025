package frc.robot.commands;

import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;

public class ActivateBase extends Command{
    ActivateBase(){
        addRequirements(/*Subsystems.coralArm,*/ Subsystems.algaeArm, Subsystems.elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
    }


    @Override
    public void end(boolean interrupted) {
        // Subsystems.coralArm.stow();
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.STOW);
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.STOW);
        super.end(interrupted);
    }
}