package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateCoralLoad extends ActivateBase{
    public ActivateCoralLoad(){this(false);}
    public ActivateCoralLoad(boolean StayUnstowed){
        super(StayUnstowed);
    }

    @Override
    public void initialize() {
        System.out.println("Moving to load coral");
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.CORAL_LOAD);
        Subsystems.coralArm.setTargetAngle(CoralArmConstants.ANGLES.LOAD);
        Subsystems.flower.extend();
    }
    
    @Override
    public void end(boolean interrupted) {
        Subsystems.flower.retract();
        super.end(interrupted);
    }
}
