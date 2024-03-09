package frc.robot.subsystems.swervedrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase{
    private CANSparkMax TopShooterMotor = new CANSparkMax(15,MotorType.kBrushless);
    private CANSparkMax BottomShooterMotor = new CANSparkMax(14,MotorType.kBrushless);
    public ShooterSubsystem(){
        TopShooterMotor.setSmartCurrentLimit(80);
        BottomShooterMotor.setSmartCurrentLimit(80);
    }

    public void shootTop(){
        TopShooterMotor.set(1);
    }

    public void shootBottom(){
        BottomShooterMotor.set(-1);
    }

    public void shootSlow(){
        TopShooterMotor.set(1);
        BottomShooterMotor.set(-1);
    }

    public void intake(){
        TopShooterMotor.set(1);
        BottomShooterMotor.set(-1);
    }

    public void stopShooter(){
        TopShooterMotor.set(0);
        BottomShooterMotor.set(0);
    }
}
