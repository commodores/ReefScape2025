package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorTwo extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;

    public ElevatorTwo() {
        // Initialize motor
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevator, MotorType.kBrushless);

        // Initialize PID controller and encoder
        pidController = elevatorMotor.getClosedLoopController();
        encoder = elevatorMotor.getEncoder();

        // Define motion profile constraints
        constraints = new TrapezoidProfile.Constraints(2.54, 2.54);

        // Initialize states
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);

        // Configure motor settings
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        config.closedLoop.p(Constants.ElevatorConstants.KP);
        config.closedLoop.i(Constants.ElevatorConstants.KI);
        config.closedLoop.d(Constants.ElevatorConstants.KD);
        config.closedLoop.velocityFF(Constants.ElevatorConstants.KFF);
        config.encoder.positionConversionFactor(Constants.ElevatorConstants.kMeterPerRevolution);

        // Apply configuration to the motor
        elevatorMotor.configure(config, PersistMode.kPersistParameters);
    }

    public void setGoal(double position) {
        goalState = new TrapezoidProfile.State(position, 0);
    }

    @Override
    public void periodic() {
        // Update the motion profile
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goalState, currentState);
        currentState = profile.calculate(0.02); // Assuming a 20ms loop time

        // Command the motor to the new position
        pidController.setReference(currentState.position, ControlType.kPosition);

        // Optionally, display encoder position on SmartDashboard
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }
}
