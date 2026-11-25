package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * For now the reason we have a Robot class is to prevent duplicate code between our Autons and Teleop classes
 * So instead of copying and pasting code between both, we just make a Team37244Robot object
 */
public class Team37244Robot {

    // === Declare OpMode Members ===
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private CRServo leftPlowServo = null;
    private CRServo rightPlowServo = null;
    private IMU imu = null;

    private final int LAUNCHER_TARGET_VELOCITY = 1165; //2496 RPM
    private final double LAUNCHER_MIN_VELOCITY = 1100; // 2357 RPM

    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    private double leftFrontDrivePower;
    private double rightFrontDrivePower;
    private double leftBackDrivePower;
    private double rightBackDrivePower;

    // === MOTOR POWER CONSTANTS ===
    final double INTAKE_POWER = 1;
    final double OUTTAKE_POWER = -1;
    final double PLOW_DEPLOYING_POWER = 1;
    final double PLOW_DEPLOY_HOLD_POWER = 0.1;
    final double PLOW_STOWING_POWER = -1;
    final double PLOW_STOW_HOLD_POWER = -0.1;

    // === STATES ===
    public enum PlowState {
        STOWING,
        DEPLOYING,
        STOW_HOLD,
        DEPLOY_HOLD,
        STOP
    }
    private PlowState currentPlowState = PlowState.STOWING;

    public enum IntakeState {
        INTAKING,
        OUTTAKING,
        STOP
    }
    private IntakeState currentIntakeState = IntakeState.STOP;

    /**
     * Initialize robot. Should be called in init() of OpMode class
     */
    public void init(HardwareMap hwMap) {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        intake = hwMap.get(DcMotor.class, "intake");
        leftPlowServo = hwMap.get(CRServo.class, "left_plow");
        rightPlowServo = hwMap.get(CRServo.class, "right_plow");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        // Intake as well
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * set Plow servos to an initial value to initialize the servo controller
         */
        leftPlowServo.setPower(0);
        rightPlowServo.setPower(0);

        // Plow Servo Directions: Positive Power is Deploy, Negative is Stow
        leftPlowServo.setDirection(CRServo.Direction.FORWARD);
        rightPlowServo.setDirection(CRServo.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        // Orientation parameters should match Control Hub on robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    /**
     * Runs intake based on state
     */
    public void runIntake() {
        // Set Power
        switch (currentIntakeState) {
            case INTAKING:
                intake.setPower(INTAKE_POWER);
                break;
            case OUTTAKING:
                intake.setPower(OUTTAKE_POWER);
                break;
            case STOP:
                intake.setPower(0);
                break;
        }
    }

    /**
     * Runs plow based on state
     */
    public void runPlow() {
        // Set Power
        switch (currentPlowState) {
            case DEPLOYING:
                leftPlowServo.setPower(PLOW_DEPLOYING_POWER);
                rightPlowServo.setPower(PLOW_DEPLOYING_POWER);
                break;
            case DEPLOY_HOLD:
                leftPlowServo.setPower(PLOW_DEPLOY_HOLD_POWER);
                rightPlowServo.setPower(PLOW_DEPLOY_HOLD_POWER);
                break;
            case STOWING:
                leftPlowServo.setPower(PLOW_STOWING_POWER);
                rightPlowServo.setPower(PLOW_STOWING_POWER);
                break;
            case STOW_HOLD:
                leftPlowServo.setPower(PLOW_STOW_HOLD_POWER);
                rightPlowServo.setPower(PLOW_STOW_HOLD_POWER);
                break;
        }
    }

    /**
     * Handles all Field Centric Drive using IMU getYaw().
     * Rotates joystick vector (x, y) by robot heading and passes it on to robotCentricDrive().
     * Therefore,this method depends on robotCentricDrive()
     * @param x left joystick x to control translation
     * @param y left joystick y to control translation
     * @param rx right joystick x to control rotation
     */
    public void fieldCentricDrive(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw();
        // We rotate the left joystick vector <x, y> by botHeading
        // Then we pass in this new rotated vector to robotCentricDrive,
        // since the vector now points in the direction from the robot's point of view.
        rx = -rx; // Strafing is inverted for some reason
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        robotCentricDrive(rotX, rotY, rx);
    }

    /**
     * Handles robot centric drive
     * Currently this method is private because we should only be using robot centric drive
     * @param forward forward power
     * @param strafe strafe (left and right) power
     * @param rotate rotational power
     */
    private void robotCentricDrive(double forward, double strafe, double rotate) {
        leftFrontDrivePower = forward + strafe + rotate;
        rightFrontDrivePower = forward - strafe - rotate;
        leftBackDrivePower = forward - strafe + rotate;
        rightBackDrivePower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontDrivePower), Math.abs(rightFrontDrivePower));
        max = Math.max(max, Math.abs(leftBackDrivePower));
        max = Math.max(max, Math.abs(rightBackDrivePower));

        if (max > 1.0) {
            leftFrontDrivePower /= max;
            rightFrontDrivePower /= max;
            leftBackDrivePower /= max;
            rightBackDrivePower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontDrivePower);
        rightFrontDrive.setPower(rightFrontDrivePower);
        leftBackDrive.setPower(leftBackDrivePower);
        rightBackDrive.setPower(rightBackDrivePower);
    }

    /**
     * Sets the power of all drive motors to 0.
     */
    public void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Sets the power of motors and mechanisms to 0.
     */
    public void stopAllMotorsAndMechanisms() {
        stopDriveMotors();

        // Other
        intake.setPower(0);
        rightPlowServo.setPower(0);
        leftPlowServo.setPower(0);
    }

    public double getLeftFrontDrivePower() {
        return leftFrontDrivePower;
    }

    public double getRightFrontDrivePower() {
        return rightFrontDrivePower;
    }

    public double getLeftBackDrivePower() {
        return leftBackDrivePower;
    }

    public double getRightBackDrivePower() {
        return rightBackDrivePower;
    }

    public void setCurrentIntakeState(IntakeState state) {
        currentIntakeState = state;
    }

    public IntakeState getCurrentIntakeState() {
        return currentIntakeState;
    }

    public String getCurrentIntakeStateAsString() {
        switch (currentIntakeState) {
            case INTAKING:
                return "INTAKING";
            case OUTTAKING:
                return "OUTTAKING";
            case STOP:
                return "STOP";
            default:
                return "UNKNOWN";
        }
    }

    public void setCurrentPlowState(PlowState state) {
        currentPlowState = state;
    }

    public PlowState getCurrentPlowState() {
        return currentPlowState;
    }

    public String getCurrentPlowStateAsString() {
        switch (currentPlowState) {
            case DEPLOYING:
                return "DEPLOYING";
            case DEPLOY_HOLD:
                return "DEPLOY_HOLD";
            case STOWING:
                return "STOWING";
            case STOW_HOLD:
                return "STOW_HOLD";
            case STOP:
                return "STOP";
            default:
                return "UNKNOWN";
        }
    }

    public double getIMUYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void resetIMUYaw() {
        imu.resetYaw();
    }
}