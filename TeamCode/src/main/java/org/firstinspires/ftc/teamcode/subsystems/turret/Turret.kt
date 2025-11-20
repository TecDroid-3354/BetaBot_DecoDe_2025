package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.positionMotorEx.PositionMotorEx

/**
 * Intended to control the [Turret] subsystem.
 * @param hardwareMap contains the information of the hardware. Configurable through Driver Hub.
 * @param telemetry to print real-time data through Driver Hub.
 */
@Suppress("JoinDeclarationAndAssignment")
class Turret(val hardwareMap: HardwareMap, val telemetry: Telemetry): SubsystemBase() {
    // Motor that controls the Turret. Motor: GoBilda 435 RPM
    private val motorController: PositionMotorEx

    // Initialization //
    init {
        // Ensure functional limits
        require(TurretConstants.Limits.maximumLimit.degrees > TurretConstants.Limits.minimumLimit.degrees)
        motorController = PositionMotorEx(
            Motor(hardwareMap, TurretConstants.Identification.turretId), turretMotorConfig)
    }

    override fun periodic() {
        // motorController.getPosition() applies the reduction, returning the position of the subsystem.
        telemetry.addData("TurretPositionDegrees", motorController.getPosition().degrees)
    }

    /**
     * Sets the angle of the subsystem. Takes into account limits defined in [TurretConstants.Limits]
     * @param angle must be the angle of the subsystem, not the motor.
     */
    fun setTurretAngle(angle: Angle) {
        val coercedAngle = Angle.fromDegrees(
            MathUtils.clamp(angle.degrees,
            TurretConstants.Limits.minimumLimit.degrees,
            TurretConstants.Limits.maximumLimit.degrees)
        )
        motorController.setPosition(coercedAngle)
    }
}