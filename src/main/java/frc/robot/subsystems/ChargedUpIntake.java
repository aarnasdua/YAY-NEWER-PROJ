package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.MotorConfigs;

public class ChargedUpIntake extends SubsystemBase {
    final private String CAN_BUS = "rio";
    final private String MOTOR_TYPE = "Falcon500";
    final private int MAX_VOLTS = 2;
    private TalonFX motor;

    /** the state of the bottom motor */
    private VoltageOut voltageOut;
    private double voltageInput;

    /**
     * The constructor
     */
    public ChargedUpIntake() {
        this.motor = new TalonFX(1, CAN_BUS);
        this.voltageInput = 0.5;
        this.voltageOut = new VoltageOut(0);
        configureMotors();
    }

    /**
     * Configure the motors
     */
    private void configureMotors() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
                .withMotorOutput(
                        MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
                .withFeedback(MotorConfigs.getFeedbackConfigs(1 / 1));

        motor.getConfigurator().apply(motorConfigs);
       
    }

    /**
     * Set the control of a specific motor
     * 
     * @param req
     *            - the control request
     */
    private void setControl(ControlRequest req) {
        if (motor.isAlive()) {
            motor.setControl(req);
        }
    }

    /**
     * Get the state of a motor (either clockwise, counterclockwise, or at rest) as
     * an int
     * 
     * @return the state of the motor: -1 = clockwise, 1 = counterclockwise, 0 = at
     *         rest
     */
    public int getMotorState() {
        double velocity = motor.getVelocity().getValueAsDouble();

        // Use 0.01 to avoid noise issue
        if (velocity > 0.01) {
            // Clockwise
            return -1;
        } else if (velocity < -0.01) {
            // Counterclockwise
            return 1;
        }

        // At rest
        return 0;
    }

    /**
     * Increase the voltage of a specific motor
     * 
     * @param input
     *            - the amount of voltage you are trying to increase (this is
     *            always positive)
     */
    public void voltageUp(double input) {
        if (input < 0)
            throw new Error("The input must be positive");
        setControl(voltageOut.withOutput(input));
    }

    /**
     * Decrease the voltage of a specific motor
     * 
     * @param input
     *            - the amount of voltage you are trying to decrease (this is
     *            always positive)
     */
    public void voltageDown(double input) {
        if (input < 0)
            throw new Error("The input must be positive");
        setControl(voltageOut.withOutput(-input));
    }

    /**
     * Stop the motor
     */
    public void stopMotor() {
        setControl(voltageOut.withOutput(0));
       
    }

    /**
     * Get the velocity of a specified motor
     */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void UpperCCW(double yAxis) {
        double volts = yAxis * MAX_VOLTS;
        setControl(voltageOut.withOutput(-volts));
    }

    public void UpperCW(double yAxis) {
        double volts = yAxis * MAX_VOLTS;
        setControl(voltageOut.withOutput(volts));
    }
}