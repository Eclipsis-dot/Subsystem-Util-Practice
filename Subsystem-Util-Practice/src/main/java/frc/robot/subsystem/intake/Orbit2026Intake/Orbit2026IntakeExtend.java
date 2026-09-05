package frc.robot.subsystem.intake.Orbit2026Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This class contains the code for Orbit intake subsystem for 2026 
 */
public class Orbit2026IntakeExtend extends SubsystemBase {

    public static class Orbit2026IntakeConstants {
        // CAN IDs
        public static final int INTAKE_TELE_MOTOR_ID = 10;
        public static final int INTAKE_ROLLER_MOTOR_ID = 11;

        // Current Limits (Amps)
        public static final double TELE_STATOR_LIMIT = 40.0;
        public static final double ROLLER_STATOR_LIMIT = 60.0;

        // Motor Settings
        public static final double INTAKE_SPEED = 0.5;
        public static final boolean kMotorInverted = true;

        // Tele Motor Reductions & Gear Ratios
        public static final double TELE_REDUCTION = 3.0;
        public static final double TELE_GEAR_RATIO = 3.0;

        // Roller Motor Reductions & Gear Ratios
        public static final double ROLLER_REDUCTION = 1.0;
        public static final double ROLLER_GEAR_RATIO = 1.0;

        // Tele Motor PID Gains
        public static final double TELE_KP = 0.1;
        public static final double TELE_KI = 0.0;
        public static final double TELE_KD = 2.0;

        // Tele Motor Feedforward Gains
        public static final double TELE_KS = 0.2;
        public static final double TELE_KV = 0.1;
        public static final double TELE_KA = 0.01;
        public static final double TELE_KG = 0.0;
    }

    private final TalonFX teleMotor = new TalonFX(IntakeConstants.INTAKE_TELE_MOTOR_ID);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_MOTOR_ID);

    private final VoltageOut voltageRequest = new VoltageOut(IntakeConstants.INTAKE_SPEED);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withPosition(0);

    private final DCMotorSim motorSim = new DCMotorSim(
        LinearSystemId.identifyPositionSystem(Units.radiansToRotations(0.12), Units.radiansToRotations(0.003)),
        DCMotor.getKrakenX60(1)
    );

    public Orbit2026IntakeExtend() {
        teleMotor.getConfigurator().apply(configureTeleMotor(new TalonFXConfiguration()));
        rollerMotor.getConfigurator().apply(configureRollerMotor(new TalonFXConfiguration()));
    }

    @Override
    public void simulationPeriodic() {
        var simState = teleMotor.getSimState();
        simState.setSupplyVoltage(12);
        
        double volts = simState.getMotorVoltage();
        motorSim.setInput(volts);
        motorSim.update(0.02);
        var rotorPos = motorSim.getAngularPositionRotations();
        var rotorVel = motorSim.getAngularVelocityRPM() / 60.0;
        simState.setRawRotorPosition(rotorPos);
        simState.setRotorVelocity(rotorVel);
    }

    public static TalonFXConfiguration configureTeleMotor(TalonFXConfiguration config) {
        config.CurrentLimits.withStatorCurrentLimit(IntakeConstants.TELE_STATOR_LIMIT).withStatorCurrentLimitEnable(true);
        config.Feedback.SensorToMechanismRatio = IntakeConstants.TELE_GEAR_RATIO;
        return config;
    }

    public static TalonFXConfiguration configureRollerMotor(TalonFXConfiguration config) {
        config.CurrentLimits.withStatorCurrentLimit(IntakeConstants.ROLLER_STATOR_LIMIT).withStatorCurrentLimitEnable(true);
        config.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEAR_RATIO;
        return config;
    }


    public Command extendAndRunRollers(double targetRotations, double rollerVolts) {
    // Both motors receive their control requests together in the same 20ms frame
        return run(() -> {
            teleMotor.setControl(positionRequest.withPosition(targetRotations));
            rollerMotor.setControl(voltageRequest.withOutput(rollerVolts));
        });
    }
                
    public static double distanceToMotorRotations(double distance) {
        double mechanismRotations = distance / IntakeConstants.DISTANCE_PER_MECHANISM_ROTATION;
        return mechanismRotations * IntakeConstants.TELE_GEAR_RATIO;
    }

    public static Command extendToLength(double distance, double rollerVolts) {
        double targetRotations = distanceToMotorRotations(distance);

        return extendAndRunRollers(targetRotations, rollerVolts);
       
    }
}