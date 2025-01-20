/** */
package frc.robot.subsystems;
import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;

public class Elevator extends SubsystemBase{

  // Define the motor and encoders
  private final SparkMax elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevator, MotorType.kBrushless);
  private SparkClosedLoopController elevatorPid = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final SparkMaxConfig config = new SparkMaxConfig();


  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(2.54, 2.54);
  private final TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints);
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;

 
 public Elevator() {


  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  
 // TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.54, 2.54);
  //TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0, 0.0);
 // TrapezoidProfile profile = new TrapezoidProfile(constraints);


    config
        .inverted(true)
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);
    //config.signals.primaryEncoderPositionPeriodMs(5);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       //double position = elevatorMotor.getEncoder().getPosition();
    
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pid(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);

   // elevatorPid.setReference(Constants.ElevatorConstants.KP, null);
   // elevatorPid.setReference(Constants.ElevatorConstants.KI, null);
   // elevatorPid.setReference(Constants.ElevatorConstants.KD, null);
    //elevatorPid.setReference(Constants.ElevatorConstants.KFF, null);

   
   elevatorMotor.getEncoder();

    config.encoder.positionConversionFactor(Constants.ElevatorConstants.kMeterPerRevolution);
    //config.closedLoop.setFeedbackDevice(RelativeEncoder);
  

   // elevatorPid.setFeedbackDevice(); 
   


  }

  protected void useState(TrapezoidProfile.State m_profile) {
    elevatorPid.setReference(m_profile.position, ControlType.kPosition);
    //elevatorPid.setReference(Constants.ElevatorConstants.KP, ControlType.kPosition);
  }

  public Command setElevatorGoalCommand(double goal) {
    m_goal = new TrapezoidProfile.State(goal, m_constraints.maxVelocity);
    return Commands.runOnce(() -> setGoal(m_goal), this);
  }

   
  public double getEncoder(){
    return elevatorEncoder.getPosition();
  }

  private void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  @Override
  public void periodic() {
    // Display current values on the SmartDashboard

    SmartDashboard.putNumber("Elevator Raw", getEncoder());
    SmartDashboard.putNumber("Elevator Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity());

    
  }
}

