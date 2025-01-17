package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import frc.robot.subsystems.Elevator;


public class Elevator extends TrapezoidProfile {


//Define Motors Encoders and Config?
private final SparkMax elevatorMotor;
private final SparkClosedLoopController m_PIDController;
private final RelativeEncoder m_relative_encoder;
private final SparkMaxConfig elevatorConfig;

private TrapezoidProfile.State goalState;
 /**Creates a new Elevator. */
public Elevator() {

    
   super(new TrapezoidProfile.Constraints(2.54, 2.54));

    elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevator, MotorType.kBrushless);
    m_PIDController = elevatorMotor.getClosedLoopController();
    m_relative_encoder = elevatorMotor.getEncoder();
    elevatorConfig = new SparkMaxConfig();
    
    elevatorConfig.inverted(true);
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.smartCurrentLimit(40);
   // elevatorConfig.configure(elevatorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

   
    elevatorConfig.closedLoop.p(Constants.ElevatorConstants.KP);
    elevatorConfig.closedLoop.i(Constants.ElevatorConstants.KI);
    elevatorConfig.closedLoop.d(Constants.ElevatorConstants.KD);
    elevatorConfig.closedLoop.velocityFF(Constants.ElevatorConstants.KFF);


   
    elevatorConfig.encoder.positionConversionFactor(Constants.ElevatorConstants.kMeterPerRevolution);
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    }

    
    public void periodic(){

        SmartDashboard.putNumber("Elevator Raw", getEncoder());

        
    }

    protected void useState(TrapezoidProfile.State setPoint){

        m_PIDController.setReference(setPoint.position,ControlType.kPosition);
    }

   public Command setElevatorGoalCommand(double goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public double getEncoder() {
        return m_relative_encoder.getPosition();

    }


}
    


