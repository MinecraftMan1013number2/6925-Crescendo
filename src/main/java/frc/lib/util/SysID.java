package frc.lib.util;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.SwerveSubsys;

public class SysID {
    private final SysIdRoutine routine;
    
    public SysID(SwerveSubsys swerveSubsys) {
        // Configure SignalLogger API through CTRE
        SignalLogger.setPath("/signal-logger-logs/");
        SignalLogger.start();
        
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        Config config = new Config();
        Mechanism mechanism = new Mechanism(null, null, swerveSubsys);

        // new Mechanism(
        //     // Tell SysId how to plumb the driving voltage to the motors.
        //     voltage -> swerveSubsys.setVoltage(voltage),
        //     // Tell SysId how to record a frame of data for each motor on the mechanism being
        //     // characterized.
        //     null,
        //     // Tell SysId to make generated commands require this subsystem, suffix test state in
        //     // WPILog with this subsystem's name ("drive")
        //     swerveSubsys);

        // routine = new SysIdRoutine(config, mechanism);
        routine = null;
    }
}
