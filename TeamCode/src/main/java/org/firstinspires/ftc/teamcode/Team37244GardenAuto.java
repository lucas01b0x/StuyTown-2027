package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// This is a simple auton where the bot shoots the preload and drives towards the loading zone

@Autonomous(name = "Team37244GardenAuto", group = "Testing")
public class Team37244GardenAuto extends LinearOpMode {
    Team37244Robot robot;

    // Autonomous timing
    private static final double AUTO_TIMEOUT_SECONDS = 29.0;

    // Timers
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Autonomous states
    private enum AutoState {
        SPIN_UP,
        SHOOT_PRELOAD,
        DRIVE_GARDEN,
        DRIVE_FORWARD,
        LOADING_ZONE_PARK,
        COMPLETE
    }
    private AutoState currentState = AutoState.DRIVE_FORWARD;

    @Override
    public void runOpMode() {
        robot = new Team37244Robot();

        robot.init(hardwareMap);

        // Ensure mechanisms start stopped
        robot.stopAllMotorsAndMechanisms();

        telemetry.addLine("Autonomous initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.stopAllMotorsAndMechanisms();
            return;
        }

        // Reset timers at the beginning of auton
        currentState = AutoState.DRIVE_FORWARD;
        autoTimer.reset();
        stateTimer.reset();

        while (opModeIsActive()) {
            // Stops before the 30-second limit
            if (autoTimer.seconds() >= AUTO_TIMEOUT_SECONDS) {
                currentState = AutoState.COMPLETE;
            }

            robot.runIntake();
            // For now, plow is always stow
            robot.setCurrentPlowState(Team37244Robot.PlowState.STOW_HOLD);
            robot.runPlow();

            switch (currentState) {
                case SPIN_UP:
                    // Spin up launcher
                    // If launcher reaches target speed, go to next state.
                    break;

                case SHOOT_PRELOAD:
                    // Keep wheel at shooting speed
                    // Run indexer
                    // Run intake
                    robot.setCurrentIntakeState(Team37244Robot.IntakeState.INTAKING);

                    if (stateTimer.seconds() >= 1) {
                        stateTimer.reset();
                        currentState = AutoState.DRIVE_GARDEN;
                    }
                    break;

                case DRIVE_GARDEN:
                    robot.fieldCentricDrive(0, 0.3, 0);
                    if (stateTimer.seconds() >= 3) {
                        stateTimer.reset();
                        currentState = AutoState.DRIVE_FORWARD;
                    }
                    break;

                case DRIVE_FORWARD:
                    robot.fieldCentricDrive(1, 0, 0);
                    if (stateTimer.seconds() >= 1) {
                        robot.stopDriveMotors();
                        stateTimer.reset();
                        currentState = AutoState.LOADING_ZONE_PARK;
                    }
                    break;

                case LOADING_ZONE_PARK:
                    robot.fieldCentricDrive(1, -0.3, 0);
                    if (stateTimer.seconds() >= 1) {
                        robot.stopDriveMotors();
                        stateTimer.reset();
                        currentState = AutoState.COMPLETE;
                    }
                    break;

                case COMPLETE:
                    robot.stopAllMotorsAndMechanisms();
                    robot.setCurrentIntakeState(Team37244Robot.IntakeState.STOP);
                    robot.setCurrentPlowState(Team37244Robot.PlowState.STOP);
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData(
                    "Autonomous Time",
                    "%.2f", autoTimer.seconds());
            telemetry.update();

            // Exit once the autonomous routine is complete
            if (currentState == AutoState.COMPLETE) {
                robot.stopAllMotorsAndMechanisms();
                break;
            }
        }

        // Final safety stop
        robot.stopAllMotorsAndMechanisms();
    }
}