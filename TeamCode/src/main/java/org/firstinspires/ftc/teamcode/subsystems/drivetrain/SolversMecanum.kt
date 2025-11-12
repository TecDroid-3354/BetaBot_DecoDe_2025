package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import LinearVelocity
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry


class SolversMecanum(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
) : SubsystemBase() {

    // Declaring motors
    lateinit var frontRightMotor: Motor
    lateinit var frontLeftMotor: Motor
    lateinit var backRightMotor: Motor
    lateinit var backLeftMotor: Motor

    // Declaring mecanum from solverslib
    var mecanum: MecanumDrive

    // Initialization code //
    init {
        motorsConfig()

        // Setting up the mecanum using the previously declared motors
        mecanum = MecanumDrive(
            frontLeftMotor, frontRightMotor,
            backLeftMotor, backRightMotor
        )
    }

    // Functional code //

    // Robot-oriented chassis speeds
    // Only takes one parameter: the chassis speeds to be used
    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        mecanum.driveRobotCentric(
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond)
    }

    // Field-oriented chassis speeds
    // Takes two parameters: chassis speeds and gyro angle in degrees, with the last one taken from the IMU
    fun setChassisSpeedsFromFieldOriented(chassisSpeeds: ChassisSpeeds, gyroAngleInDegrees: Double) {
        mecanum.driveFieldCentric(
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            gyroAngleInDegrees)
    }

    // Setup code //
    fun motorsConfig() {
        // Configuring motors according to their revolutions per minute
        frontRightMotor = Motor(hardwareMap, MecanumConstants.Ids.frontRightId, GoBILDA.RPM_312)
        frontLeftMotor = Motor(hardwareMap, MecanumConstants.Ids.frontLeftId, GoBILDA.RPM_312)
        backRightMotor = Motor(hardwareMap, MecanumConstants.Ids.backRightId, GoBILDA.RPM_312)
        backLeftMotor = Motor(hardwareMap, MecanumConstants.Ids.backLeftId, GoBILDA.RPM_312)
    }
}