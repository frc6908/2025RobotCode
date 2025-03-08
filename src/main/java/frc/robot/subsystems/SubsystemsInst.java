package frc.robot.subsystems;

public class SubsystemsInst {
    //public Drivetrain drivetrain;
    public Vision vision;
    //public ArmSubsystem armSubsystem;
    //public ClimberSubsystem climberSubsystem;
   
   
    private static SubsystemsInst inst;

    private SubsystemsInst() {
        //drivetrain = new Drivetrain();
        vision = new Vision();
        //armSubsystem = new ArmSubsystem();
        //climberSubsystem = new ClimberSubsystem();
    }

    public static SubsystemsInst getInst () {
        if(inst == null) inst = new SubsystemsInst();

        return inst;

    }
    
}