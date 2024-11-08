package frc.lib.util;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveSubsys;

public class SysID {
    private final SysIdRoutine sysIdRoutine;
    
    public SysID(SwerveSubsys swerveSubsys) {
        // Configure SignalLogger API through CTRE
        SignalLogger.setPath("/signal-logger-logs/");
        SignalLogger.start();
        
        SysIdRoutine.Config config = new SysIdRoutine.Config(
            null,               // Use default ramp rate (1 V/s)
            Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null,                // Use default timeout (10 s)
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
        );
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
            (voltage) -> swerveSubsys.setVoltage(voltage.in(Units.Volts)),
            null,
            swerveSubsys
        );

        sysIdRoutine = new SysIdRoutine(config, mechanism);
    }

    /**
     * Quasistatic:
     * In this test, the mechanism is gradually sped-up such that the voltage
     * corresponding to acceleration is negligible (hence, “as if static”)
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Dynamic:
     * In this test, a constant ‘step voltage’ is given to the mechanism,
     * so that the behavior while accelerating can be determined
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
