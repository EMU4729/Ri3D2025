package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateCoralL3 extends ActivateBase{
    ActivateCoralL3(){
        addRequirements(/*Subsystems.coralArm,*/ Subsystems.elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.CORAL_L3);
        //Subsystems.coralArm.setTargetAngle(CoralArmConstants.ANGLES.UNLOAD);
    }
}
