package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// This is a simple auton where the bot shoots the preload and drives towards the loading zone

@Autonomous(name = "Team37244GardenAuto", group = "Testing")
public class Team37244GardenAuto extends LinearOpMode {
    // Autonomous timing
    private static final double AUTO_TIMEOUT_SECONDS = 29.0;

    private static final double STRAFE_POWER = 0.3;
    private static final double STRAFE_TIME_SECONDS = 1.0;

    private static final double FORWARD_POWER = -0.6;
    private static final double FORWARD_TIME_SECONDS = 1;

    // Hardware
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private IMU imu = null;

    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Timers
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Autonomous states
    private enum AutoState {
        DRIVE_FORWARD,
        LOADING_ZONE_PARK,
        COMPLETE
    }

    private AutoState currentState = AutoState.DRIVE_FORWARD;

    @Override
    public void runOpMode() {
        // Map hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

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

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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
        currentState = AutoState.DRIVE_FORWARD;
        autoTimer.reset();
        stateTimer.reset();

        while (opModeIsActive()) {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Stops before the 30-second limit
            if (autoTimer.seconds() >= AUTO_TIMEOUT_SECONDS) {
                currentState = AutoState.COMPLETE;
            }

            switch (currentState) {
                case DRIVE_FORWARD:
                    fieldCentricDrive(1, 0, 0, botHeading);
                    if (stateTimer.seconds() >= FORWARD_TIME_SECONDS) {
                        stopDriveMotors();
                        stateTimer.reset();
                        currentState = AutoState.LOADING_ZONE_PARK;
                    }
                    break;

                case LOADING_ZONE_PARK:
                    fieldCentricDrive(1, -0.3, 0, botHeading);
                    if (stateTimer.seconds() >= FORWARD_TIME_SECONDS) {
                        stopDriveMotors();
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

    void fieldCentricDrive(double x, double y, double rx, double botHeading) {
        // We rotate the left joystick vector <x, y> by botHeading
        // Then we pass in this new rotated vector to robotCentricDrive,
        // since the vector now points in the direction from the robot's point of view.
        rx = -rx; // Strafing is inverted for some reason
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        robotCentricDrive(rotX, rotY, rx);
    }

    void robotCentricDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void stopAllMotorsAndMechanisms() {
        stopDriveMotors();
        // Other motors here when we have more mechanisms
    }
}