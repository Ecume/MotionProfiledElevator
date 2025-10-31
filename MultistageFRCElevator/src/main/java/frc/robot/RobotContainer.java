package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AprilTagOffsets;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TagOffset;

import frc.robot.subsystems.ElevatorSubsystem;


public class RobotContainer {

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    //public final LEDSubsystem led_sybsystem = new LEDSubsystem();



    private final Field2d field;
    private final SendableChooser<Command> autoChooser;

    public XboxController driver = new XboxController(0);
    public static XboxController gunner = new XboxController(1);

    public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        NamedCommands.registerCommand("L4CORAL", new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L3CORAL", new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L2CORAL", new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L1CORAL", new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("LoadStation", new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        NamedCommands.registerCommand("Boost", new InstantCommand(() -> elevatorSubsystem.boost(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("Intake", new WaitCommand(0.75));
        //               .andThen(new InstantCommand(() -> recordAttempt()))


        autoChooser = AutoBuilder.buildAutoChooser("Straight Auto");

        // Control diagram: https://docs.google.com/drawings/d/1NsJOx6fb6KYHW6L8ZeuNtpK3clnQnIA9CD2kQHFL0P0/edit?usp=sharing

        new JoystickButton(gunner, Buttons.Y).onTrue(new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.B).onTrue(new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.A).onTrue(new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.X).onTrue(new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem));
        //optional
        //new POVButton(gunner, Buttons.POV_LEFT).onTrue(new InstantCommand(() -> elevatorSubsystem.dynamic_L4_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));

     
   
    

}
}