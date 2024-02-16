package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class PivotSubsys extends SubsystemBase {

    private TalonFX pivotMotor = new TalonFX(15);

    private PositionDutyCycle intakePivotPosition = new PositionDutyCycle(0);
    private DutyCycleOut intakePivotPercentOutput = new DutyCycleOut(0);

    public PivotSubsys() {
        configIntakePivotMotor();
    }

    public void intakePivot(double position) {

        intakePivotPosition.Position = position;
        pivotMotor.setControl(intakePivotPosition);

    }

    public void intakePivotPercentOutput(double percentOutput) {

        intakePivotPercentOutput.Output = percentOutput;
        pivotMotor.setControl(intakePivotPercentOutput);

    }

    public void resetIntakePivot() {

        pivotMotor.setPosition(0);

    }

    public void configIntakePivotMotor() {

        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.getConfigurator().apply(Robot.ctreConfigs.intakePivotFXConfig);
        resetIntakePivot();

    }

    public double getIntakePivotPosition() {

        return pivotMotor.getPosition().getValueAsDouble();

    }

    public double getIntakePivotRotorPosition() {

        return pivotMotor.getRotorPosition().getValueAsDouble();

    }

    public enum PivotState {

        NONE(Double.MAX_VALUE),
        GROUND(PivotConstants.INTAKE_PIVOT_GROUND),
        SOURCE(PivotConstants.INTAKE_PIVOT_SOURCE),
        AMP(PivotConstants.INTAKE_PIVOT_AMP),
        STOW(PivotConstants.INTAKE_PIVOT_STOW);

        public double intakeSetpoint;
        public double intakeVelocity;

        private PivotState(double pivotSetpoint) {
            intakeSetpoint = pivotSetpoint;
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot Position", getIntakePivotPosition());
        SmartDashboard.putNumber("Intake Pivot Rotor Position", getIntakePivotRotorPosition());
    }
}