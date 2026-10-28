package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

// This is a simple auton where the bot shoots the preload and drives towards the loading zone

@Autonomous(name = "Team37244StarterBotAuto", group = "StarterBot")
public class Team37244StarterBotAuto extends LinearOpMode {
    // Launcher settings
    private static final double LAUNCHER_TARGET_VELOCITY = 1165; // 2496 RPM
    private static final double LAUNCHER_MIN_VELOCITY = 1085; // 2325 RPM

    // Autonomous timing
    private static final double AUTO_TIMEOUT = 29.0;
    private static final double FEED_TIME = 4.0;

    private static final double STRAFE_POWER = 0.3;
    private static final double STRAFE_TIME = 3.0;

    private static final double FORWARD_POWER = -0.6;
    private static final double FORWARD_TIME = 2.5;

    // Hardware
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private DcMotor intake;
    private DcMotorEx launcher;

    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    // Timers
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Autonomous states
    private enum AutoState {
        SPIN_UP,
        FEED_PRELOAD,
        DRIVE_FORWARD,
        LOADING_ZONE,
        COMPLETE
    }

    private AutoState currentState = AutoState.SPIN_UP;

    @Override
    public void runOpMode() {
        // Map hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // Configure drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake drive motors when power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure launcher
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Configure servo directions
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        // Ensure mechanisms start stopped
        stopAllMotorsAndMechanisms();

        telemetry.addLine("Autonomous initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAllMotorsAndMechanisms();
            return;
        }

        // Reset timers at the beginning of auton
        currentState = AutoState.SPIN_UP;
        autoTimer.reset();
        stateTimer.reset();

        while (opModeIsActive()) {

            // Stops before the 30-second limit
            if (autoTimer.seconds() >= AUTO_TIMEOUT) {
                currentState = AutoState.COMPLETE;
            }

            switch (currentState) {
                case SPIN_UP:
                    // Spin up launcher
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                    if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                        stateTimer.reset();
                        currentState = AutoState.FEED_PRELOAD;
                    }
                    break;

                case FEED_PRELOAD:
                    // Keep wheel at shooting speed
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                    // Run indexer
                    windmillServo.setPower(1.0);

                    // Run intake motor and both intake servos
                    intake.setPower(0.5);
                    leftIntakeServo.setPower(0.5);
                    rightIntakeServo.setPower(0.5);

                    if (stateTimer.seconds() >= FEED_TIME) {
                        windmillServo.setPower(0);
                        intake.setPower(0);
                        leftIntakeServo.setPower(0);
                        rightIntakeServo.setPower(0);
                        launcher.setVelocity(0);
                        stateTimer.reset();
                        currentState = AutoState.LOADING_ZONE;
                    }
                    break;

                case LOADING_ZONE:
                    boolean strafeFinished = timedMecanumDrive(0, STRAFE_POWER, 0, STRAFE_TIME);
                    if (strafeFinished) currentState = AutoState.DRIVE_FORWARD;
                    break;

                case DRIVE_FORWARD:
                    // Drive forward
                    boolean forwardFinished = timedMecanumDrive(FORWARD_POWER, 0, 0, FORWARD_TIME);
                    if (forwardFinished) {
                        stateTimer.reset();
                        currentState = AutoState.COMPLETE;
                    }
                    break;

                case COMPLETE:
                    stopAllMotorsAndMechanisms();
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData(
                    "Autonomous Time",
                    "%.2f", autoTimer.seconds());
            telemetry.addData(
                    "Launcher Velocity",
                    "%.0f", launcher.getVelocity());
            telemetry.update();

            // Exit once the autonomous routine is complete
            if (currentState == AutoState.COMPLETE) {
                stopAllMotorsAndMechanisms();
                break;
            }
        }

        // Final safety stop
        stopAllMotorsAndMechanisms();
    }

    // Existing timed mecanum drive function
    private boolean timedMecanumDrive(double forward, double strafe, double rotate, double holdSeconds) {
        // Mecanum wheel power calculations
        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        // Normalize powers so none exceed magnitude 1
        double maxPower = Math.max(1.0,
                Math.max(
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
                )
        );

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;

        if (stateTimer.seconds() >= holdSeconds) {
            stopDriveMotors();
            return true;
        }

        // Apply power to drive motors
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        return false;
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void stopAllMotorsAndMechanisms() {
        stopDriveMotors();
        if (intake != null) intake.setPower(0);
        if (launcher != null) launcher.setVelocity(0);
        if (windmillServo != null) windmillServo.setPower(0);
        if (leftIntakeServo != null) leftIntakeServo.setPower(0);
        if (rightIntakeServo != null) rightIntakeServo.setPower(0);
    }
}
