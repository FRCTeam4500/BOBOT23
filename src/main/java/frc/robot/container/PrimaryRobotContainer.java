package frc.robot.container;


import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autonomous.routines.*;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.dashboard.DashboardNumberDisplay;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.HardwareArmFactory;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.camera.CameraImpl;
import frc.robot.subsystem.camera.HardwareCameraFactory;
import frc.robot.subsystem.climber.ClimberConstants;
import frc.robot.subsystem.climber.command.ClimberSetAngleCommand;
import frc.robot.subsystem.intake.HardwareIntakeFactory;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.lights.Lights;
import frc.robot.subsystem.loader.HardwareLoaderFactory;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderSetOutputCommand;
import frc.robot.subsystem.shooter.HardwareShooterFactory;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.shooter.command.DumpBallCommand;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinUpCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand.ControlMode;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.turret.HardwareTurretFactory;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.turret.command.TurretDefaultCommand;
import frc.robot.subsystem.vision.HardwareVisionFactory;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.util.VisionDistanceCalculator;
import frc.robot.utility.ControllerInfo;
import frc.robot.utility.PolarVelocityCalculator;


import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.sequence.ClimbToHighBar;
import frc.robot.subsystem.climber.HardwareClimberFactory;
import frc.robot.subsystem.climber.command.ClimberSetAngleCommand;
import frc.robot.subsystem.camera.HardwareCameraFactory;
import frc.robot.subsystem.camera.Camera;

public class PrimaryRobotContainer implements RobotContainer{

    //Initialize subsystems
    // private Arm arm = HardwareArmFactory.makeArm();
    // private Intake intake = HardwareIntakeFactory.makeIntake();
    // private Loader loader = HardwareLoaderFactory.makeLoader();
    // private Shooter shooter = HardwareShooterFactory.makeShooter();
    private PathFollowingSwerve swerve = HardwareSwerveFactory.makeSwerve();
    // private Turret turret = HardwareTurretFactory.makeTurret();
    // private Vision vision = HardwareVisionFactory.makeVision();
    // private CameraImpl camOne = HardwareCameraFactory.makeCameraInstance();
    // private Climber climber = HardwareClimberFactory.makeClimber();
    //private CameraInstance camera = HardwareCameraFactory.makeCameraInstance();
    // private Lights turretLights = new Lights(1, 0, 60);

    //Initialize Joysticks and Buttons
    private Joystick driveStick = new Joystick(0);
    private ControllerInfo info = new ControllerInfo();

    private JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, 1);
    private JoystickButton switchDriveModeRobotCentric = new JoystickButton(driveStick, 4);
    private JoystickButton alignSwerveToAngle = new JoystickButton(driveStick, 8);
    private JoystickButton alignSwerveReverse = new JoystickButton(driveStick, 7);
    private JoystickButton resetGyro = new JoystickButton(driveStick, 10);
    private JoystickButton limitSwerveSpeed = new JoystickButton(driveStick, 2);
    private JoystickButton noForwardButton = new JoystickButton(driveStick, 9);

    // private POVButton uPad = new POVButton(driveStick, 0);
    // private POVButton urPad = new POVButton(driveStick, 45);
    // private POVButton rPad = new POVButton(driveStick, 90);
    // private POVButton drPad = new POVButton(driveStick, 135);
    // private POVButton dPad = new POVButton(driveStick, 180);
    // private POVButton dlPad = new POVButton(driveStick, 225);
    // private POVButton lPad = new POVButton(driveStick, 270);
    // private POVButton ulPad = new POVButton(driveStick, 315);

    private Joystick controlStick = new Joystick(1);

    // private JoystickButton intakeButton = new JoystickButton(controlStick, 1);
    // private JoystickButton shootButton = new JoystickButton(controlStick, 4);
    // private JoystickButton reverseLoadButton = new JoystickButton(controlStick, 2);
    // private JoystickButton dumpButton = new JoystickButton(controlStick, 3);


    // private JoystickButton climberZero = new JoystickButton(controlStick, 6);
    // private JoystickButton climberUp = new JoystickButton(controlStick, 7);
    // private JoystickButton climberClimb = new JoystickButton(controlStick, 5);

    // private JoystickButton noTurret = new JoystickButton(controlStick, 8);
    
    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    // private PolarVelocityCalculator calculator;
    private TriModeSwerveCommand swerveCommand;

    //private Lights.Routines defaultRoutine = Lights.Routines.blueorange;
    private Lights.Routines defaultRoutine = Lights.Routines.rainbow;

    boolean isPlacing = false;
    boolean isIntaking = false;

    public PrimaryRobotContainer(){
        configureControls();
        // configureClimber();
        configureSwerve();
        // configureIntakeAndCameraAndArm();
        // configureShooting();
        // configureAutonomous();
        // configureLights();
    }

    void configureControls(){
        info.xSensitivity = 4;
        info.ySensitivity = 4;
        info.zSensitivity = 3.5;
        info.xDeadzone = 0.2;
        info.yDeadzone = 0.2;
        info.zDeadzone = 0.2;
        Shuffleboard.getTab("Driver Controls").add("Driver Controls", info);
        Shuffleboard.getTab("Driver Controls").add("Messages", messages);
        //Shuffleboard.getTab("Driver Controls").add("Intake Camera", camera);

        // Shuffleboard.getTab("Driver Controls")
        //         .add("Placing", isPlacing)
        //         .withWidget(BuiltInWidgets.kBooleanBox);

        // Shuffleboard.getTab("Driver Controls")
        //         .add("Intaking", isIntaking)
        //         .withWidget(BuiltInWidgets.kBooleanBox);
    }

    
    

    // void configureClimber() {

    //     Shuffleboard.getTab("Climber").add("climber", climber);
        

    //     climberZero.toggleOnTrue(new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_ZERO_ANGLE));

    //     climberUp.toggleOnTrue(
    //             new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_UP_ANGLE)
    //                     .alongWith(new InstantCommand(() -> isPlacing = true))
    //                     .alongWith(new WaitCommand(2))
    //                     .andThen(new InstantCommand(() -> isPlacing = false))
    //                     .alongWith(new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_ZERO_ANGLE))
    //     );

    //     climberClimb.toggleOnTrue(
    //             new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_CLIMB_ANGLE)
    //                     .alongWith(new InstantCommand(() -> {isIntaking = true;}))
    //                     .alongWith(new WaitCommand(1.5))
    //                     .andThen(new InstantCommand(() -> {isIntaking = false;}))
    //                     .alongWith(new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_ZERO_ANGLE))
    //     );

    // }

    void configureSwerve() {
        swerveCommand = new TriModeSwerveCommand(swerve, driveStick, info, messages);
        swerveCommand.controlMode = ControlMode.FieldCentric;

        switchDriveModeRobotCentric.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;})); 
        switchDriveModeRobotCentric.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        lockSwerveRotationButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.lockRotation = true;}));
        lockSwerveRotationButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.lockRotation = false;}));

        noForwardButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.RobotCentric; swerveCommand.noForward = true;}));
        noForwardButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric; swerveCommand.noForward = false;}));

        alignSwerveToAngle.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        alignSwerveToAngle.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        alignSwerveReverse.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = Math.PI;}));
        alignSwerveReverse.toggleOnFalse(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;}));

        limitSwerveSpeed.toggleOnTrue(new InstantCommand(() -> {swerveCommand.limitSpeed = true;}));
        limitSwerveSpeed.toggleOnFalse(new InstantCommand(() -> {swerveCommand.limitSpeed = false;}));

        resetGyro.toggleOnTrue(new InstantCommand(() -> {swerve.resetRobotAngle();}));

        // uPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        // urPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 45;}));
        // rPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 90;}));
        // drPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 135;}));
        // dPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 180;}));
        // dlPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        // lPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));
        // ulPad.toggleOnTrue(new InstantCommand(() -> {swerveCommand.controlMode = ControlMode.AlignToAngle; swerveCommand.targetAngle = 0;}));

        swerve.setDefaultCommand(swerveCommand);
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);

        // calculator = new PolarVelocityCalculator(swerve, vision, turret);
        // Shuffleboard.getTab("Swerve").add("Polar Calulator", calculator);

    }

    // void configureIntakeAndCameraAndArm() {
    //     intakeButton.toggleOnTrue(new ArmSetAngleCommand(arm, ArmConstants.ARM_DOWN_ANGLE)
    //             .alongWith(new IntakeRunCommand(intake, IntakeConstants.intakeRunSpeed)));
    //     intakeButton.toggleOnFalse(
    //         new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE).alongWith(
    //         new IntakeRunCommand(intake, 0))
                        
    //     );

    //     // configure camera
    //     //camOne.start();
        
        
    //     Shuffleboard.getTab("Intake").add("Intake", intake);
    //     Shuffleboard.getTab("Intake").add("Arm", arm);
    //     //Shuffleboard.getTab("Intake").add("Intake Camera", camOne);
    // }

    // void configureShooting() {
    //     turret.setDefaultCommand(new TurretDefaultCommand(turret, vision, calculator));
    //     //shooter.setDefaultCommand(new ShooterContinuousRunCommand(shooter, () -> 0));
    //     //loader.setDefaultCommand(new LoaderRunCommand(loader, 0));

    //     //manual shooting
    //     //ShooterControl control = new ShooterControl(10000, 50);
    //     //Command shootCommand = new ManualShootingCommand(shooter, vision, loader, control);
    //     //shootButton.whenPressed(shootCommand);
    //     //shootButton.whenReleased(() -> {resetShooting();});
    //     //Shuffleboard.getTab("Shooting").add("Shooter control", control);
    //     //Automated shooting
    //     shootButton.toggleOnTrue(new AutomatedShootingCommand(shooter, vision, loader, turret, calculator).alongWith(new InstantCommand(() -> {swerveCommand.limitSpeed = true;})));
    //     shootButton.toggleOnFalse(new InstantCommand(() -> {resetShooting(); swerveCommand.limitSpeed = false;}));

    //     noTurret.toggleOnTrue(new InstantCommand(() -> {turret.setEnabled(false);}));


    //     //Run shooter and loader in reverse
    //     Command reverseLoadCommand = new ParallelCommandGroup(new ShooterSpinUpCommand(shooter, new ShooterControl(20000,50)),
    //             new LoaderSetOutputCommand(loader, -1));
    //     reverseLoadButton.toggleOnTrue(reverseLoadCommand);
    //     reverseLoadButton.toggleOnFalse(new InstantCommand(() -> {resetShooting();}));

    //     Command dumpCommand = new DumpBallCommand(turret, shooter, vision, loader).alongWith(new InstantCommand(() -> turretLights.setCurrentRoutine(Lights.Routines.orangeflash)));
    //     dumpButton.toggleOnTrue(dumpCommand);
    //     dumpButton.toggleOnFalse(new InstantCommand(() -> {resetShooting(); resetLights();}));

    //     //Shuffleboard
    //     ShuffleboardTab tab = Shuffleboard.getTab("Shooting");
    //     //tab
    //     tab.add("Shooter", shooter);
    //     tab.add("Turret", turret);
    //     tab.add("Loader", loader);
    //     tab.add("Distance", new DashboardNumberDisplay("Distance", () -> VisionDistanceCalculator.calculateDistance(vision)));
    // }

    // void configureLights() {
    //     turretLights.setCurrentRoutine(Lights.Routines.blueorbit);
    //     Shuffleboard.getTab("Light").add("light", turretLights);
    // }

    /*void configureAutonomous() {
        autonChooser.setDefaultOption("First Ball", new FirstBallAuto(swerve, arm, shooter, intake, vision, loader, turret, turretLights, calculator));
        autonChooser.addOption("Triangle Auto", new TriangleAuto(swerve, arm, intake, shooter, vision, loader, turret, turretLights, calculator));
        autonChooser.addOption("Console Auto", new ConsoleAuto(swerve, arm, shooter, intake, vision, loader, turret, turretLights,  calculator));
        autonChooser.addOption("Second Ball", new SecondBallAuto(swerve, arm, shooter, intake, vision, loader, turret, turretLights, calculator));
        autonChooser.addOption("Third Ball", new ThirdBallAuto(swerve, arm, shooter, intake, vision, loader, turret, turretLights, calculator));
        Shuffleboard.getTab("Driver Controls").add("Autonomous Route", autonChooser);
    }*/


    // void resetShooting() {
    //     if (shooter.getCurrentCommand() != null) {
    //         shooter.getCurrentCommand().cancel();
    //     }
    //     shooter.setSpeed(0);
    //     loader.setOutput(0);
    //     turret.setEnabled(true);
    // }

    // void resetLights() {
    //     turretLights.setCurrentRoutine(defaultRoutine);
    // }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
    @Override
    public void teleopInit() {
        Command auton = autonChooser.getSelected();
        if (auton != null){
            auton.cancel();
        }
        // resetShooting();
        // resetLights();
        // new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE).schedule(); //deploy the arm
    }
}
