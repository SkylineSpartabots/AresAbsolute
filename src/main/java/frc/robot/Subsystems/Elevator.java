// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private static Elevator instance;

  private TalonFX follower;
  private TalonFX leader;
  private VoltageOut voltOutput;
  private TorqueCurrentFOC torqueOutput;

  private boolean holdPosition = false;

  // sysid
      private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(3), // Reduce dynamic step voltage to 4 V to prevent brownout
            Units.Seconds.of(5),        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdElevator_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            this::setVoltage,
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutine;
    
    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }


  public static Elevator getInstance(){
    if(instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  public enum ElevatorState {
    L1(17.5), //bad
    L2(21.7),
    L3(36.3),
    L4(61),
    GROUND(0.11), //bad
    A1(10.5), //bad
    A2(26), //bad
    SOURCE(2.35);
    //48.1 should be max
    private double encoderPosition;

    private ElevatorState(double encoderPosition){
      this.encoderPosition = encoderPosition;
    }

    public double getEncoderPosition(){
      return encoderPosition;
    }
  }

  public Elevator() {
    leader = new TalonFX(Constants.HardwarePorts.elevatorLeaderId);
    follower = new TalonFX(Constants.HardwarePorts.elevatorFollowerId);
    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);
    configMotor(leader, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
    configMotor(follower, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);

    // follower.setControl(new Follower(Constants.HardwarePorts.elevatorLeaderId, false));

    voltOutput = new VoltageOut(0).withEnableFOC(true);
    torqueOutput = new TorqueCurrentFOC(0);
  }

  private void configMotor(TalonFX motor, InvertedValue direction, NeutralModeValue neutralMode){
    // motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.Inverted = direction;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.elevatorContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.elevatorPeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits = currentLimitsConfigs;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motor.getConfigurator().apply(config);
    motor.getPosition().setUpdateFrequency(50);
    motor.getStatorCurrent().setUpdateFrequency(50);
    motor.getMotorVoltage().setUpdateFrequency(50);
    motor.optimizeBusUtilization();

    Slot0Configs configuration = new Slot0Configs();
    configuration.kG = 0.25;
    configuration.kI = 0.15;
    configuration.kP = 0.13;

    motor.getConfigurator().apply(configuration);
     // motor.optimizeBusUtilization();
  }

  public void setPosition(double position){
    leader.setControl(new PositionVoltage(position).withSlot(0).withUpdateFreqHz(50));
    follower.setControl(new PositionVoltage(position).withSlot(0).withUpdateFreqHz(50));
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public void setTorqueOutput(double output){
    leader.setControl(torqueOutput.withOutput(output));
    follower.setControl(torqueOutput.withOutput(output));
  }

  public double getCurrent(){
    return leader.getStatorCurrent().getValueAsDouble();
  }

  public void stop(){
    leader.setControl(voltOutput.withOutput(0));
    follower.setControl(voltOutput.withOutput(0));
  }

  public void setSpeed(double speed){
    holdPosition = false;
    leader.set(speed);
    follower.set(speed);
  }

  public void setVoltage(double voltage){
    holdPosition = false;
    leader.setControl(voltOutput.withOutput(voltage));
    follower.setControl(voltOutput.withOutput(voltage));
  }

  public void setVoltage(Voltage voltage){
    holdPosition = false;
    leader.setControl(voltOutput.withOutput(voltage));
    follower.setControl(voltOutput.withOutput(voltage));
  }

  public double getVelocity(){
    return leader.getVelocity().getValueAsDouble();
  }

  public double getLeaderVoltage(){
    return leader.getMotorVoltage().getValueAsDouble();
  }

  public double getFollowerVoltage(){
    return follower.getMotorVoltage().getValueAsDouble();
  }

  public double getAcceleration() {
    return leader.getAcceleration().getValueAsDouble();
  }

  public void brake(){
    leader.setControl(new PositionVoltage(leader.getPosition().getValueAsDouble()).withEnableFOC(true).withSlot(0));
    follower.setControl(new PositionVoltage(follower.getPosition().getValueAsDouble()).withEnableFOC(true).withSlot(0));

  }

  public void zeroPosition() {
    leader.setPosition(0);
    follower.setPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("elevator position", getPosition());
    // SmartDashboard.putNumber("elevator stator current", getCurrent());
  }
}
