// Package main is an example of a custom viam server.

package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/go-daq/canbus"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	goutils "go.viam.com/utils"
	viamutils "go.viam.com/utils"

	"go.viam.com/rdk/components/base"
	_ "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// Modal limits
const PEDAL_MAX = 100.0
const STEERANGLE_MAX = 25.0
const SPEED_LIMP_HOME = 20.0 // Max speed (throttle) if a limp home condition is active

var model = resource.NewModel("intermode", "modal", "omnidirectional")

func main() {
	goutils.ContextualMain(mainWithArgs, logging.NewDebugLogger("intermodeBaseModule"))
}

// /////////////
// Fail-Safe //
// /////////////
const commsTimeoutIntervalMs = 1000 // If it has been at least this long since last command received, execute containment
var commsTimeout time.Time
var commsTimeoutEnable = false // Enable or disable comms timeout
// Presently changed based off of received command style

type driveCommand struct {
	Accelerator   float64
	Brake         float64
	SteeringAngle float64
	Gear          byte
	DriveMode     byte
	SteerMode     byte
}

type intermodeBase struct {
	resource.Named

	trackWidthMm         float64
	wheelCircumferenceMm float64
	gearRatio            float64
	geometries           []spatialmath.Geometry

	name   string
	logger logging.Logger

	canTxSocket             canbus.Socket
	nextCommandCh           chan canbus.Frame
	isMoving                atomic.Bool
	activeBackgroundWorkers sync.WaitGroup
	cancel                  func()
}
type axleCommand struct {
	canId         uint32
	rightSpeed    float64
	leftSpeed     float64
	Brake         float64
	SteeringAngle float64
}
type modalCommand interface {
	toFrame(logger logging.Logger) canbus.Frame
}

var (
	gears = map[string]byte{
		gearPark:          0,
		gearReverse:       1,
		gearNeutral:       2,
		gearDrive:         3,
		gearEmergencyStop: 4,
	}
	steerModes = map[string]byte{
		steerModeFrontWheelSteer: 0,
		steerModeRearWheelSteer:  1,
		steerModeFourWheelSteer:  2,
		steerModeCrabSteering:    3,
	}
	driveModes = map[string]byte{
		driveModeFourWheelDrive:  0,
		driveModeFrontWheelDrive: 1,
		driveModeRearWheelDrive:  2,
		driveModeIndAped:         3,
	}

	emergencyCmd = driveCommand{
		Accelerator:   0,
		Brake:         1,
		SteeringAngle: 0,
		Gear:          gears[gearEmergencyStop],
		DriveMode:     driveModes[driveModeFourWheelDrive],
		SteerMode:     steerModes[steerModeFourWheelSteer],
	}
	stopCmd = driveCommand{
		Accelerator:   0,
		Brake:         1,
		SteeringAngle: 0,
		Gear:          gears[gearPark],
		DriveMode:     driveModes[driveModeFourWheelDrive],
		SteerMode:     steerModes[steerModeFourWheelSteer],
	}
)

const (
	channel = "can0"
	// channel = "vcan0"

	// CAN IDs
	kulCanIdCmdDrive uint32 = 0x220
	kulCanIdCmdAxleF uint32 = 0x222
	kulCanIdCmdAxleR uint32 = 0x223

	// Vehicle properties
	kVehicleWheelbaseMm  = 680
	kVehicleTrackwidthMm = 515
	kVehicleSeparation   = (kVehicleWheelbaseMm + kVehicleTrackwidthMm) / 2

	// Wheel properties
	kWheelRadiusMm        float64 = 125
	kWheelCircumferenceMm float64 = 2 * math.Pi * kWheelRadiusMm
	kWheelEncoderBits     int     = 12
	kWheelTicksPerRev     int     = 1 << kWheelEncoderBits

	kMagnitudeMaxX float64 = 1
	kMagnitudeMaxY float64 = 1

	// Limits and defaults
	kLimitCurrentMax  = 5                                                                     // Maximum motor current
	kLimitSpeedMaxKph = 5                                                                     // Max speed in KPH
	kGearRatio        = 3.0                                                                   // Gear ratio of motor to wheel
	kLimitSpeedMaxRpm = kLimitSpeedMaxKph * 1000000 / kWheelCircumferenceMm / 60 * kGearRatio // Max speed in RPM
	kDefaultCurrent   = kLimitCurrentMax                                                      // Used for straight and spin commands

	kNumBitsPerByte = 8

	kCanIdTelemWheelSpeedId   uint32 = 0x241
	kCanIdTelemBatteryPowerId uint32 = 0x250
	kCanIdTelemBatteryStateId uint32 = 0x251

	// Keys
	gearPark          = "park"
	gearReverse       = "reverse"
	gearNeutral       = "neutral"
	gearDrive         = "drive"
	gearEmergencyStop = "emergency_stop"

	steerModeFrontWheelSteer = "front-wheel-steer"
	steerModeRearWheelSteer  = "rear-wheel-steer"
	steerModeFourWheelSteer  = "four-wheel-steer"
	steerModeCrabSteering    = "crab-steering"

	driveModeFourWheelDrive  = "four-wheel-drive"
	driveModeFrontWheelDrive = "front-wheel-drive"
	driveModeRearWheelDrive  = "rear-wheel-drive"
	driveModeIndAped         = "independent-aped-drive"
)

var (
	// Constants that must be calculated at runtime
	// Distance wheel tirepatch is from center of vehicle
	kVehicleTirePatchRadiusMm = math.Sqrt(math.Pow(kVehicleWheelbaseMm/2, 2) + math.Pow(kVehicleTrackwidthMm/2, 2))
	// Circumference of circle with radius of tirepatch distance from center of vehicle
	kVehicleTirePatchCircumferenceMm = 2 * math.Pi * kVehicleTirePatchRadiusMm

	// Wheel revolutions to vehicle rotations
	kWheelRevPerVehicleRev = kVehicleTirePatchCircumferenceMm / kWheelCircumferenceMm
)

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) (err error) {
	registerBase()
	modalModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}
	modalModule.AddModelFromRegistry(ctx, base.API, model)

	err = modalModule.Start(ctx)
	defer modalModule.Close(ctx)

	if err != nil {
		return err
	}
	<-ctx.Done()
	return nil
}

// calculateAccelAndBrakeBytes returns the intermode specific acceleration and brake bytes for the given
// acceleration percentage.
func calculateAccelAndBrakeBytes(accelPct float64, brakePct float64) []byte {
	var speedLimit = SPEED_LIMP_HOME
	var ok = false

	if !ok {
		speedLimit = SPEED_LIMP_HOME
	}
	if speedLimit >= 0.0 { // Limit has been received from controller
		accelPct = accelPct * speedLimit
	} else { // No limit received, apply limp home mode
		accelPct = accelPct * SPEED_LIMP_HOME
	}

	accelPct = math.Abs(accelPct)
	brakePct = brakePct * PEDAL_MAX

	// TODO: Remove after the Modal properly handles two pedal
	if brakePct > 0 {
		accelPct = 0.0
	}

	accelBytes := uint16(accelPct / 0.0625) // intermode scalar
	brakeBytes := uint16(brakePct / 0.0625)

	retBytes := make([]byte, 4)
	binary.LittleEndian.PutUint16(retBytes[0:2], accelBytes)
	binary.LittleEndian.PutUint16(retBytes[2:4], brakeBytes)
	return retBytes
}

/*
 * Convert a drive command to a CAN frame
 */
func (cmd *driveCommand) toFrame(logger logging.Logger) canbus.Frame {
	frame := canbus.Frame{
		ID:   kulCanIdCmdDrive,
		Data: make([]byte, 0, 8),
		Kind: canbus.SFF,
	}

	steeringAngleBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(steeringAngleBytes, 0)

	frame.Data = append(frame.Data, calculateAccelAndBrakeBytes(cmd.Accelerator, cmd.Brake)...)
	frame.Data = append(frame.Data, steeringAngleBytes...) // Steering hard-coded to 0 as turning is handled by the wheels
	frame.Data = append(frame.Data, cmd.Gear|(cmd.DriveMode<<4), cmd.SteerMode)

	logger.Debugw("frame", "data", frame.Data)

	return frame
}

/*
 * Convert an axle command to a CAN frame
 */
func (cmd *axleCommand) toFrame(logger logging.Logger) canbus.Frame {
	frame := canbus.Frame{
		ID:   cmd.canId,
		Data: make([]byte, 8),
		Kind: canbus.SFF,
	}

	// TODO: Remove magic number scalars
	rightSpeedBytes := uint16(cmd.rightSpeed / 0.0078125)
	leftSpeedBytes := uint16(cmd.leftSpeed / 0.0078125)
	brakeBytes := uint16(cmd.Brake / 0.0625)
	steeringAngleBytes := uint16(cmd.SteeringAngle / 0.0078125)

	binary.LittleEndian.PutUint16(frame.Data[0:2], rightSpeedBytes)
	binary.LittleEndian.PutUint16(frame.Data[2:4], leftSpeedBytes)
	binary.LittleEndian.PutUint16(frame.Data[4:6], brakeBytes)
	binary.LittleEndian.PutUint16(frame.Data[6:8], steeringAngleBytes)

	// logger.Debugw("frame", "data", frame.Data)

	return frame
}

/**
 * Sets the next base CAN command
 */
func (base *intermodeBase) setNextCommand(ctx context.Context, cmd modalCommand) error {
	if err := ctx.Err(); err != nil {
		base.logger.Infow("Ctx err in setNextCommand()")
		return err
	}

	select {
	case <-ctx.Done():
		base.logger.Infow("Ctx done in setNextCommand()")
		return ctx.Err()
	case base.nextCommandCh <- cmd.toFrame(base.logger):
	}

	return nil
}

// Close cleanly closes the base.
func (base *intermodeBase) Close(ctx context.Context) error {
	base.setNextCommand(ctx, &stopCmd)
	base.cancel()
	base.activeBackgroundWorkers.Wait()

	return nil
}

// // publishThread continuously sends the current drive command over the canbus.
func publishThread(
	ctx context.Context,
	socket canbus.Socket,
	nextCommandCh chan canbus.Frame,
	logger logging.Logger,
) {
	defer socket.Close()
	commsTimeout = time.Now().Add(commsTimeoutIntervalMs * time.Millisecond)

	driveFrame := (&stopCmd).toFrame(logger)
	var frame canbus.Frame

	for {
		if ctx.Err() != nil {
			return
		}
		select {
		case <-ctx.Done():
		case frame = <-nextCommandCh:
			if frame.ID == kulCanIdCmdDrive {
				// new drive command will replace the existing drive command and be sent every 10ms.
				driveFrame = frame
				commsTimeout = time.Now().Add(commsTimeoutIntervalMs * time.Millisecond)
			} else {
				// non-drive commands should be sent immediately, but retain current drive command for base heartbeating.
				if _, err := socket.Send(frame); err != nil {
					logger.Errorw("non-drive command send error", "error", err)
				}
			}
		case <-time.After(10 * time.Millisecond):
		}
		if commsTimeoutEnable && time.Now().After(commsTimeout) {
			driveFrame = (&emergencyCmd).toFrame(logger)
		}
		if _, err := socket.Send(driveFrame); err != nil {
			logger.Errorw("Drive command send error", "error", err)
		}
	}
}

/*
	Intermode Differential Base Implementation
	Every method will set the next command for the publish loop to send over the command bus forever.
*/

// MoveStraight moves the base forward the given distance and speed.
// TODO: Actually implement for differential. Commented section does a fixed APED that isn't ideal
func (base *intermodeBase) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	base.logger.Warnw("MoveStraight not implemented")
	// // Speed
	// var speedNegative = mmPerSec < 0
	// var rpsDes = mmPerSec / kWheelCircumferenceMm
	// var rpmDesMagnitude = math.Abs(rpsDes * 60)
	// rpmDesMagnitude = math.Min(float64(rpmDesMagnitude), kLimitSpeedMaxRpm)

	// // Distance
	// var distanceNegative = distanceMm < 0
	// var distanceRev = float64(distanceMm) / kWheelCircumferenceMm
	// var encoderMagnitude = math.Abs(float64(kWheelTicksPerRev) * distanceRev)
	// // Add 0.5 for rounding purposes
	// var encoderValue = int32(encoderMagnitude + 0.5)

	// cmd := driveCommand{
	// 	Accelerator:   0.5,
	// 	Brake:         0,
	// 	SteeringAngle: 0,
	// 	Gear:          gears[gearDrive],
	// 	DriveMode:     driveModes[driveModeIndAped],
	// 	SteerMode:     steerModes[steerModeFourWheelSteer],
	// }

	// if true == distanceNegative || true == speedNegative {
	// 	cmd.Gear = gears[gearReverse]
	// }

	// if err := base.setNextCommand(ctx, &cmd); err != nil {
	// 	return err
	// }
	// base.isMoving.Store(true) // TODO: Replace with feedback info

	// defer func() {
	// 	base.setNextCommand(ctx, &stopCmd)
	// 	base.isMoving.Store(false)
	// }()

	// var waitSeconds = float64(distanceMm)/math.Abs(mmPerSec) + 1.5
	// if !viamutils.SelectContextOrWait(ctx, time.Duration(waitSeconds * float64(time.Second))) {
	// 	return ctx.Err()
	// }

	return nil
}

// Spin spins the base by the given angleDeg and degsPerSec.
// TODO: Actually implement for differential. Commented section does a fixed APED that isn't ideal
func (base *intermodeBase) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	base.logger.Warnw("Spin not implemented")
	// // Speed
	// var speedNegative = degsPerSec < 0
	// var rpmDesMagnitude = math.Abs(degsPerSec / 360 * 60 * kWheelRevPerVehicleRev)
	// rpmDesMagnitude = math.Min(rpmDesMagnitude, kLimitSpeedMaxRpm)

	// // Angle
	// var angleNegative = angleDeg < 0
	// var encoderMagnitude = math.Abs(angleDeg/360*kWheelRevPerVehicleRev) * float64(kWheelTicksPerRev)
	// var encoderValue = int32(encoderMagnitude)

	// cmd := driveCommand{
	// 	Accelerator:   0.5,
	// 	Brake:         0,
	// 	SteeringAngle: 0,
	// 	Gear:          gears[gearDrive],
	// 	DriveMode:     driveModes[driveModeIndAped],
	// 	SteerMode:     steerModes[steerModeFourWheelSteer],
	// }

	// if true == distanceNegative || true == speedNegative {
	// 	cmd.Gear = gears[gearReverse]
	// }

	// if err := base.setNextCommand(ctx, &cmd); err != nil {
	// 	return err
	// }
	// base.isMoving.Store(true) // TODO: Replace with feedback info

	// defer func() {
	// 	base.setNextCommand(ctx, &stopCmd)
	// 	base.isMoving.Store(false)
	// }()

	// var waitSeconds = float64(distanceMm)/math.Abs(mmPerSec) + 1.5
	// if !viamutils.SelectContextOrWait(ctx, time.Duration(waitSeconds * float64(time.Second))) {
	// 	return ctx.Err()
	// }

	return nil
}

// SetPower sets the linear and angular [-1, 1] drive power.
func (base *intermodeBase) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Some vector components do not apply to a 2D base
	if 0 != linear.X {
		base.logger.Warnw("Linear X command non-zero and has no effect")
	}
	if 0 != linear.Z {
		base.logger.Warnw("Linear Z command non-zero and has no effect")
	}
	if 0 != angular.X {
		base.logger.Warnw("Angular X command non-zero and has no effect")
	}
	if 0 != angular.Y {
		base.logger.Warnw("Angular Y command non-zero and has no effect")
	}

	var linearMagnitude = linear.Y
	// Angular multiplied by 0.5 because that is the max single-linear-direction magnitude
	var rpmDesFr = linearMagnitude + angular.Z
	var rpmDesFl = linearMagnitude - angular.Z
	var rpmDesRr = linearMagnitude + angular.Z
	var rpmDesRl = linearMagnitude - angular.Z
	var rpmMax = math.Max(math.Max(rpmDesFr, rpmDesFl), math.Max(rpmDesRr, rpmDesRl))

	if rpmMax > 1 {
		rpmDesFr /= rpmMax
		rpmDesFl /= rpmMax
		rpmDesRr /= rpmMax
		rpmDesRl /= rpmMax
	}

	var driveCmd = driveCommand{
		Accelerator:   0,
		Brake:         0,
		SteeringAngle: 0,
		Gear:          gears[gearDrive],
		DriveMode:     driveModes[driveModeIndAped],
		SteerMode:     steerModes[steerModeFourWheelSteer],
	}
	var axleCmd = axleCommand{
		rightSpeed:    0.0,
		leftSpeed:     0.0,
		Brake:         0.0,
		SteeringAngle: 0.0,
	}
	var frontCmd, rearCmd = axleCmd, axleCmd

	// TODO: Switch to actually using speed instead of a percentage
	//		 Currently treating this as an Aped command at the base side
	frontCmd.canId = kulCanIdCmdAxleF
	frontCmd.rightSpeed = rpmDesFr * 100.0
	frontCmd.leftSpeed = rpmDesFl * 100.0
	rearCmd.canId = kulCanIdCmdAxleR
	rearCmd.rightSpeed = rpmDesRr * 100.0
	rearCmd.leftSpeed = rpmDesRl * 100.0

	if 0 > linearMagnitude {
		driveCmd.Gear = gears[gearReverse]
	}

	if err := base.setNextCommand(ctx, &driveCmd); err != nil {
		base.logger.Errorw("Error setting SetPower command", "error", err)
		return err
	}
	if err := base.setNextCommand(ctx, &frontCmd); err != nil {
		base.logger.Errorw("Error setting SetPower command", "error", err)
		return err
	}
	if err := base.setNextCommand(ctx, &rearCmd); err != nil {
		base.logger.Errorw("Error setting SetPower command", "error", err)
		return err
	}

	base.isMoving.Store(true) // TODO: Replace with feedback info

	return nil
}

// SetVelocity sets the linear (mmPerSec) and angular (degsPerSec) velocity.
// TODO: Actually implement for differential
func (base *intermodeBase) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Print received command
	//	Not a debug message to avoid activating all of the debug messages
	base.logger.Infow("SetVelocity with ",
		"linear.X", linear.X,
		"linear.Y", linear.Y,
		"linear.Z", linear.Z,
		"angular.X", angular.X,
		"angular.Y", angular.Y,
		"angular.Z", angular.Z,
	)

	if linear.Norm() == 0 && angular.Norm() == 0 {
		base.logger.Debug("received a SetVelocity command of linear 0,0,0, and angular 0,0,0, stopping base")
		return base.Stop(ctx, nil)
	}

	// Some vector components do not apply to a 2D base
	if 0 != linear.X {
		base.logger.Warnw("Linear X command non-zero and has no effect")
	}
	if 0 != linear.Z {
		base.logger.Warnw("Linear Z command non-zero and has no effect")
	}
	if 0 != angular.X {
		base.logger.Warnw("Angular X command non-zero and has no effect")
	}
	if 0 != angular.Y {
		base.logger.Warnw("Angular Y command non-zero and has no effect")
	}

	// TODO: Add support for four wheel differential
	rpmDesLeft, rpmDesRight := base.velocityMath(linear.Y, angular.Z)

	// Limit wheel RPM to +/-kLimitSpeedMaxRpm
	rpmDesLeft = math.Min(math.Max(rpmDesLeft, -kLimitSpeedMaxRpm), kLimitSpeedMaxRpm)
	rpmDesRight = math.Min(math.Max(rpmDesRight, -kLimitSpeedMaxRpm), kLimitSpeedMaxRpm)

	// Convert RPM to KPH
	//	Temporary until the base can handle RPM directly
	kphDesLeft, kphDesRight := base.rpmToKph(rpmDesLeft, rpmDesRight)

	var driveCmd = driveCommand{
		Accelerator:   0,
		Brake:         0,
		SteeringAngle: 0,
		Gear:          gears[gearDrive],
		DriveMode:     driveModes[driveModeIndAped],
		SteerMode:     steerModes[steerModeFourWheelSteer],
	}
	var axleCmd = axleCommand{
		rightSpeed:    0.0,
		leftSpeed:     0.0,
		Brake:         0.0,
		SteeringAngle: 0.0,
	}
	var frontCmd, rearCmd = axleCmd, axleCmd

	// TODO: Switch to actually using speed instead of a percentage
	//		 Currently treating this as an Aped command at the base side
	frontCmd.canId = kulCanIdCmdAxleF
	frontCmd.rightSpeed = kphDesRight
	frontCmd.leftSpeed = kphDesLeft
	rearCmd.canId = kulCanIdCmdAxleR
	rearCmd.rightSpeed = kphDesRight
	rearCmd.leftSpeed = kphDesLeft

	// TODO: Block this on the base side for independent wheel control
	// if 0 > linear.Y {
	// 	driveCmd.Gear = gears[gearReverse]
	// }

	if err := base.setNextCommand(ctx, &driveCmd); err != nil {
		base.logger.Errorw("Error setting SetVelocity command", "error", err)
		return err
	}
	if err := base.setNextCommand(ctx, &frontCmd); err != nil {
		base.logger.Errorw("Error setting SetVelocity command", "error", err)
		return err
	}
	if err := base.setNextCommand(ctx, &rearCmd); err != nil {
		base.logger.Errorw("Error setting SetVelocity command", "error", err)
		return err
	}

	base.logger.Infow("SetVelocity return")

	return nil
}

// calcualtes wheel rpms from overall base linear and angular movement velocity inputs.
// From Viam wheeled_base RDK implementation
func (base *intermodeBase) velocityMath(mmPerSec, degsPerSec float64) (float64, float64) {
	// Base calculations
	linearVelocity := mmPerSec
	wheelRadius := base.wheelCircumferenceMm / (2.0 * math.Pi)
	trackWidth := base.trackWidthMm

	angularVelocity := degsPerSec / 180 * math.Pi
	leftAngularVelocity := (linearVelocity / wheelRadius) - (trackWidth * angularVelocity / (2 * wheelRadius))
	rightAngularVelocity := (linearVelocity / wheelRadius) + (trackWidth * angularVelocity / (2 * wheelRadius))

	// RPM = revolutions (unit) * deg/sec * (1 rot / 2pi deg) * (60 sec / 1 min) = rot/min
	rpmL := (leftAngularVelocity / (2 * math.Pi)) * 60
	rpmR := (rightAngularVelocity / (2 * math.Pi)) * 60

	return rpmL, rpmR
}

// Converts RPM to KPH
func (base *intermodeBase) rpmToKph(rpmL, rpmR float64) (float64, float64) {
	// Converts RPM to KPH
	kphL := rpmL * base.wheelCircumferenceMm / 1000.0 * 60.0 / 1000000.0
	kphR := rpmR * base.wheelCircumferenceMm / 1000.0 * 60.0 / 1000000.0

	return kphL, kphR
}

// Stop stops the base. It is assumed the base stops immediately.
func (base *intermodeBase) Stop(ctx context.Context, extra map[string]interface{}) error {
	base.isMoving.Store(false)
	base.logger.Infow("Stopping")
	ret := base.setNextCommand(ctx, &stopCmd)
	base.logger.Infow("Stopped")
	return ret
}

// DoCommand executes additional commands beyond the Base{} interface. For this rover that includes door open and close commands.
func (base *intermodeBase) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// TODO: expand this function to change steering/gearing modes.
	name, ok := cmd["command"]
	if !ok {
		return nil, errors.New("missing 'command' value")
	}
	switch name {
	case "set_prnd":
		prndRaw, ok := cmd["prnd"]
		if !ok {
			return nil, errors.New("prnd must be set to a byte corresponding to park|reverse|neutral|drive|estop")
		}
		// TODO: Figure out how to get this to arrive as a byte/int
		prnd, ok := prndRaw.(float64)
		if !ok {
			return nil, errors.New(fmt.Sprintf("prnd value must be an int but is type %T", prndRaw))
		}

		gearFound := false
		for _, v := range gears {
			if v == byte(prnd) {
				gearFound = true
			}
		}
		if !gearFound {
			return nil, errors.New("prnd value must be an int corresponding to park|reverse|neutral|drive|estop")
		}

		return map[string]interface{}{"return": fmt.Sprintf("set_prnd command processed: %f", prnd)}, nil

	default:
		return nil, fmt.Errorf("no such command: %s", name)
	}
}

func (i *intermodeBase) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return i.geometries, nil
}

func (i *intermodeBase) Name() resource.Name {
	return resource.Name{
		API:    base.API,
		Remote: "test",
		Name:   "modal",
	}
}

func (i *intermodeBase) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}

func (i *intermodeBase) Properties(ctx context.Context, extra map[string]interface{}) (base.Properties, error) {
	return base.Properties{
		WidthMeters:              kVehicleTrackwidthMm / 1000.0,
		WheelCircumferenceMeters: kWheelCircumferenceMm / 1000.0,
	}, nil
}

func (base *intermodeBase) IsMoving(ctx context.Context) (bool, error) {
	return base.isMoving.Load(), nil
}

// newBase creates a new base that underneath the hood sends canbus frames via
// a 10ms publishing loop.
func newBase(conf resource.Config, logger logging.Logger) (base.Base, error) {
	var geometries = []spatialmath.Geometry{}
	if conf.Frame != nil {
		frame, err := conf.Frame.ParseConfig()
		if err != nil {
			return nil, err
		}
		geometries = append(geometries, frame.Geometry())
	}

	socketSend, err := canbus.New()
	if err != nil {
		return nil, err
	}

	if err := socketSend.Bind(channel); err != nil {
		return nil, err
	}

	socketRecv, err := canbus.New()
	if err != nil {
		return nil, err
	}

	if err := socketRecv.Bind(channel); err != nil {
		return nil, err
	}

	cancelCtx, cancel := context.WithCancel(context.Background())
	iBase := &intermodeBase{
		name:          conf.Name,
		canTxSocket:   *socketSend,
		nextCommandCh: make(chan canbus.Frame),
		cancel:        cancel,
		logger:        logger,
		geometries:    geometries,

		Named:                conf.ResourceName().AsNamed(),
		trackWidthMm:         kVehicleTrackwidthMm,
		wheelCircumferenceMm: kWheelCircumferenceMm,
		gearRatio:            kGearRatio,
	}
	iBase.isMoving.Store(false)

	iBase.activeBackgroundWorkers.Add(1)
	viamutils.ManagedGo(func() {
		publishThread(cancelCtx, iBase.canTxSocket, iBase.nextCommandCh, logger)
	}, iBase.activeBackgroundWorkers.Done)

	return iBase, nil
}

// helper function to add the base's constructor and metadata to the component registry, so that we can later construct it.
func registerBase() {
	resource.RegisterComponent(
		base.API,
		model,
		resource.Registration[resource.Resource, resource.NoNativeConfig]{Constructor: func(
			ctx context.Context,
			deps resource.Dependencies,
			conf resource.Config,
			logger logging.Logger,
		) (resource.Resource, error) {
			return newBase(conf, logger)
		}})
}
