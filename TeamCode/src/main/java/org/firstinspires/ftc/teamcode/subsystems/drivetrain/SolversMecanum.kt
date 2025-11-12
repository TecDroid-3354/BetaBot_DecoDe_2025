package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import LinearVelocity
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.Motor.GoBILDA
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.robotcore.external.Telemetry


class SolversMecanum(val hw: HardwareMap, val telemetry: Telemetry) : SubsystemBase() {

    lateinit var frontRightMotor: Motor
    lateinit var frontLeftMotor: Motor
    lateinit var backRightMotor: Motor
    lateinit var backLeftMotor: Motor

    lateinit var mecanum: MecanumDrive

    init {
        motorsConfig()

        mecanum = MecanumDrive(
            frontLeftMotor, frontRightMotor,
            backLeftMotor, backRightMotor
        )
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        mecanum.driveRobotCentric(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond)
    }

    fun setChassisSpeedsFromFieldOriented(chassisSpeeds: ChassisSpeeds, gyroAngleInDegrees: Double) {
        mecanum.driveFieldCentric(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, gyroAngleInDegrees)
    }

    fun motorsConfig() {
        frontRightMotor = Motor(hw, MecanumConstants.Ids.frId, GoBILDA.RPM_312)
        frontLeftMotor = Motor(hw, MecanumConstants.Ids.flId, GoBILDA.RPM_312)
        backRightMotor = Motor(hw, MecanumConstants.Ids.brId, GoBILDA.RPM_312)
        backLeftMotor = Motor(hw, MecanumConstants.Ids.blId, GoBILDA.RPM_312)

    }
}