package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command{
    private Shooter shooter;
    private double speed;

    public ShooterCommand(Shooter s_shooter, double speed){
        shooter = s_shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted){
        shooter.shooterOff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
