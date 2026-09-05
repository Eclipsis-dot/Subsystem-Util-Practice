package frc.robot.subsystem.elevator.Nomad2025OffseasonClimberLogicVariations;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Nomad2025OffseasonClimber extends SubsystemBase 
{
    public class Nomad2025OffseasonClimberConstants 
    {

        public static final int LIFT_CLIMBER_MOTOR_ID_LEAD = 12;
        public static final int LIFT_CLIMBER_ENCODER_ID_FOLLOWER = 15;
        public static final int MIN_LENGTH_ROTATIONS = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kVelocity = 0;
        public static final double kAcceleration = 0;

        public static final int kReduction = 3;
        public static final int kCANID = 32;

        public static final double kStatorCurrentLimit = 100;
        public static final boolean kMotorInverted = true;

    }

    private final TalonFX elevatorMotor = new TalonFX(
            Nomad2025OffseasonClimberConstants.LIFT_CLIMBER_MOTOR_ID_LEAD);
    private final TalonFX followerMotor = new TalonFX(
            Nomad2025OffseasonClimberConstants.LIFT_CLIMBER_ENCODER_ID_FOLLOWER);



    public final PositionVoltage positionRequest = new PositionVoltage(0).withPosition(0);

    private final DCMotorSim motorSim = new DCMotorSim(
            LinearSystemId.createElevatorSystem(
                    DCMotor.getKrakenX60(2), // 2 motors driving the mechanism
                    10.0, // Carriage mass in kg
                    Units.inchesToMeters(0.75), // Drum/sprocket radius in meters
                    12.0 // Gear reduction ratio (e.g., 12:1)
            ),

            DCMotor.getKrakenX60(2)

    );

    public Nomad2025OffseasonClimber() 
    {
        var elevatorConfig = new TalonFXConfiguration();

        elevatorMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        elevatorMotor.getConfigurator().apply(configureLiftingMotor(elevatorConfig));

        var followerConfig = new TalonFXConfiguration();

        followerMotor.getConfigurator().refresh(followerConfig);
        followerMotor.getConfigurator().apply(configureLiftingMotor(followerConfig));


        follower.setControl(new Follower(Nomad2025OffseasonClimberConstants.LIFT_CLIMBER_MOTOR_ID_LEAD, false));

        setDefaultCommand(this.hold());

        if(RobotBase.isReal())
        {
            liftingMotor.setPosition(Nomad2025OffseasonClimberConstants.MIN_LENGTH_ROTATIONS);
        }
        else
        {
         sim.setState(VecBuilder.fill(Nomad2025OffseasonClimberConstants.MIN_LENGTH_, 0));
        }
        m_setpointSig.setUpdateFrequency(50);
        setDefaultCommand(hold());

    }

    public Command home() {
        return runOnce(()->liftingMotor.setPosition(ElevatorConstants.MIN_LENGTH_ROTATIONS));
    }



    private TalonFXConfiguration configureLiftingMotor(TalonFXConfiguration config)
    {
        config.slot0.kP = Nomad2025OffseasonClimberConstants.kP;
        config.slot0.kI = Nomad2025OffseasonClimberConstants.kI;
        config.slot0.kD = Nomad2025OffseasonClimberConstants.kD;
        config.slot0.kS = Nomad2025OffseasonClimberConstants.kS;
        config.slot0.kG = Nomad2025OffseasonClimberConstants.kG;
        config.slot0.kV = Nomad2025OffseasonClimberConstants.kV;
        config.slot0.kA = Nomad2025OffseasonClimberConstants.kA;

        config.statorCurrentLimit = Nomad2025OffseasonClimberConstants.kStatorCurrentLimit;
        config.motorInverted = Nomad2025OffseasonClimberConstants.kMotorInverted;

        return config;
    }

    
}
