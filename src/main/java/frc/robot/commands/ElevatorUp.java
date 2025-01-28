package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class ElevatorUp extends InstantCommand{
    private final Elevator m_elevator;
    public ElevatorUp(Elevator elevator) {
         m_elevator = elevator;
         addRequirements(m_elevator);
    }
    
    @Override
    public void initialize() {
        m_elevator.setPositionGoal(.2); //TODO find the right set-goal, might be 24
    }
}