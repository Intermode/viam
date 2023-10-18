// Package main is an example of a custom viam server.

package main

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/edaniels/golog"
	"github.com/go-daq/canbus"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	goutils "go.viam.com/utils"
	viamutils "go.viam.com/utils"

	"go.viam.com/rdk/components/base"
	_ "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/components/base/kinematicbase"
	"go.viam.com/rdk/spatialmath"
)

// Modal limits
const PEDAL_MAX = 100.0
const STEERANGLE_MAX = 25.0
const SPEED_LIMP_HOME = 20.0 // Max speed (throttle) if a limp home condition is active

var model = resource.NewModel("intermode", "modal", "omnidirectional")

func main() {
	goutils.ContextualMain(mainWithArgs, golog.NewDevelopmentLogger("intermodeOmniBaseModule"))
}

///////////////
// Telemetry //
///////////////
var telemetryLock = sync.RWMutex{}

var (
	telemetry = map[string]interface{}{
		telemGearDesired:   byte(0),
		telemSpeed:         math.NaN(),
		telemSpeedLimit:    float64(-1),
		telemSteerAngle:    -1,
		telemStateOfCharge: -1,
	}
)

func telemSet(key string, value interface{}) {
	telemetryLock.Lock()
	defer telemetryLock.Unlock()
	telemetry[key] = value
}

///////////////
// Fail-Safe //
///////////////
const commsTimeoutIntervalMs = 1000 // If it has been at least this long since last command received, execute containment
var commsTimeout time.Time
var commsTimeoutEnable = false		// Enable or disable comms timeout
									// Presently changed based off of received command style


type (
	interModeBase = struct {
		name                    string
		canTxSocket             canbus.Socket
		nextCommandCh           chan canbus.Frame
		isMoving                atomic.Bool
		activeBackgroundWorkers sync.WaitGroup
		cancel                  func()
		logger                  golog.Logger
		geometries              []spatialmath.Geometry
	}
	driveCommand  = struct {
		Accelerator   float64
		Brake         float64
		SteeringAngle float64
		Gear          byte
		DriveMode     byte
		SteerMode     byte
	}
	axleCommand = struct {
		rightSpeed    float64
		leftSpeed     float64
		Brake         float64
		SteeringAngle float64
	}
	modalCommand = interface {
		toFrame(logger golog.Logger) canbus.Frame
	}
)

var (
	gears = map[string]byte{
		gearPark:          	0,
		gearReverse:       	1,
		gearNeutral:       	2,
		gearDrive:         	3,
		gearEmergencyStop: 	4,
	}
	steerModes = map[string]byte{
		steerModeFrontWheelSteer: 	0,
		steerModeRearWheelSteer:  	1,
		steerModeFourWheelSteer:  	2,
		steerModeCrabSteering:    	3,
	}
	driveModes = map[string]byte{
		driveModeFourWheelDrive: 	0,
		driveModeFrontWheelDrive:  	1,
		driveModeRearWheelDrive:  	2,
		driveModeIndAped:   		3,
	}

	emergencyCmd = driveCommand{
		Accelerator:   	0,
		Brake:         	1,
		SteeringAngle: 	0,
		Gear:          	gears[gearEmergencyStop],
		DriveMode:	   	driveModes[driveMode4WD],
		SteerMode:     	steerModes[steerModeFourWheelSteer],
	}
	stopCmd = driveCommand{
		Accelerator:   	0,
		Brake:         	1,
		SteeringAngle: 	0,
		Gear:          	gears[gearPark],
		DriveMode:	   	driveModes[driveMode4WD],
		SteerMode:     	steerModes[steerModeFourWheelSteer],
	}
)

const (
	channel = "can0"
	// channel = "vcan0"
	
	// CAN IDs
	kulCanIdDriveCmd uint32 = 0x220

	// Vehicle properties
	kVehicleWheelbaseMm  = 709.684
	kVehicleTrackwidthMm = 528.580
	kVehicleSeparation   = (kVehicleWheelbaseMm + kVehicleTrackwidthMm) / 2

	// Wheel properties
	kWheelRadiusMm        float64 = 77
	kWheelCircumferenceMm float64 = 2 * math.Pi * kWheelRadiusMm
	kWheelEncoderBits     int     = 12
	kWheelTicksPerRev     int     = 1 << kWheelEncoderBits

	kMagnitudeMaxX float64 = 1
	kMagnitudeMaxY float64 = 1

	// Limits and defaults
	kLimitCurrentMax  = 5                                                        // Maximum motor current
	kLimitSpeedMaxKph = 2.5                                                      // Max speed in KPH
	kLimitSpeedMaxRpm = kLimitSpeedMaxKph * 1000000 / kWheelCircumferenceMm / 60 // Max speed in RPM
	kDefaultCurrent   = kLimitCurrentMax                                         // Used for straight and spin commands

	kNumBitsPerByte = 8

	kCanIdTelemWheelSpeedId   uint32 = 0x241
	kCanIdTelemBatteryPowerId uint32 = 0x250
	kCanIdTelemBatteryStateId uint32 = 0x251
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

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) (err error) {
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

// helper function to add the base's constructor and metadata to the component registry, so that we can later construct it.
func registerBase() {
	resource.RegisterComponent(
		base.API,
		model,
		resource.Registration[resource.Resource, resource.NoNativeConfig]{Constructor: func(
			ctx context.Context,
			deps resource.Dependencies,
			conf resource.Config,
			logger golog.Logger,
		) (resource.Resource, error) {
			return newBase(conf, logger)
		}})
}

// newBase creates a new base that underneath the hood sends canbus frames via
// a 10ms publishing loop.
func newBase(conf resource.Config, logger golog.Logger) (base.Base, error) {
	geometries, err := kinematicbase.CollisionGeometry(conf.Frame)
    if err != nil {
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

	_, cancel := context.WithCancel(context.Background())
    iBase := &intermodeOmniBase{
        name:        conf.Name,
        canTxSocket: *socketSend,
		nextCommandCh: make(chan canbus.Frame),
        cancel:      cancel,
        logger:      logger,
        geometries: geometries,
    }
	iBase.isMoving.Store(false)

	iBase.activeBackgroundWorkers.Add(1)
	viamutils.ManagedGo(func() {
		publishThread(cancelCtx, iBase.canTxSocket, iBase.nextCommandCh, logger)
	}, iBase.activeBackgroundWorkers.Done)

	return iBase, nil
}

/*
 * Convert a drive command to a CAN frame
 */
func (cmd *driveCommand) toFrame(logger golog.Logger) canbus.Frame {
	frame := canbus.Frame{
		ID:   kulCanIdDriveCmd,
		Data: make([]byte, 0, 8),
		Kind: canbus.SFF,
	}

	steeringAngleBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(steeringAngleBytes, 0)

	frame.Data = append(frame.Data, calculateAccelAndBrakeBytes(cmd.Accelerator, cmd.Brake)...)
	frame.Data = append(frame.Data, steeringAngleBytes...)	// Steering hard-coded to 0 as turning is handled by the wheels
	frame.Data = append(frame.Data, cmd.Gear, cmd.DriveMode, cmd.SteerMode)

	logger.Debugw("frame", "data", frame.Data)

	return frame
}

/**
 * Sets the next base CAN command
 */
func (base *intermodeBase) setNextCommand(ctx context.Context, cmd modalCommand) error {
	if err := ctx.Err(); err != nil {
		return err
	}

	select {
		case <-ctx.Done():
			return ctx.Err()
		case base.nextCommandCh <- cmd.toFrame(base.logger):
	}

	return nil
}

// // publishThread continuously sends the current drive command over the canbus.
func publishThread(
	ctx context.Context,
	socket canbus.Socket,
	nextCommandCh chan canbus.Frame,
	logger golog.Logger,
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
func (base *intermodeOmniBase) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Some vector components do not apply to a 2D base
	if 0 != linear.Y {
		base.logger.Warnw("Linear Y command non-zero and has no effect")
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

	var linearMagnitude = linear.X
	// Angular multiplied by 0.5 because that is the max single-linear-direction magnitude
	var rpmDesFr = math.Min(math.Sin(-1*math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude + angular.Z
	var rpmDesFl = math.Min(math.Sin(math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude - angular.Z
	var rpmDesRr = math.Min(math.Sin(math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude + angular.Z
	var rpmDesRl = math.Min(math.Sin(-1*math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude - angular.Z
	var rpmMax = math.Max(math.Max(rpmDesFr, rpmDesFl), math.Max(rpmDesRr, rpmDesRl))

	if rpmMax > 1 {
		rpmDesFr /= rpmMax
		rpmDesFl /= rpmMax
		rpmDesRr /= rpmMax
		rpmDesRl /= rpmMax
	}

	var driveCmd := driveCommand{
		Accelerator:   0,
		Brake:         0,
		SteeringAngle: 0,
		Gear:          gears[gearDrive],
		DriveMode:     driveModes[driveModeIndAped],
		SteerMode:     steerModes[steerModeFourWheelSteer],
	}
	var axleCmd = axleCommand{
		rightSpeed:    float64,
		leftSpeed:     float64,
		Brake:         float64,
		SteeringAngle: float64,
	}
	var frontCmd, rearCmd = axleCmd, axleCmd

	// TODO: Switch to actually using speed instead of a percentage
	//		 Currently treating this as an Aped command at the base side
	frontCmd.rightSpeed = rpmDesFr * 100.0
	frontCmd.leftSpeed = rpmDesFl * 100.0
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
func (base *intermodeOmniBase) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	base.logger.Warnw("SetVelocity not implemented")

	// base.isMoving.Store(true) // TODO: Replace with feedback info

	// // Some vector components do not apply to a 2D base
	// if 0 != linear.Z {
	// 	base.logger.Warnw("Linear Z command non-zero and has no effect")
	// }
	// if 0 != angular.X {
	// 	base.logger.Warnw("Angular X command non-zero and has no effect")
	// }
	// if 0 != angular.Y {
	// 	base.logger.Warnw("Angular Y command non-zero and has no effect")
	// }

	// var rpmDesMagnitudeLinX = math.Abs(linear.X / kWheelCircumferenceMm * 60)
	// rpmDesMagnitudeLinX = math.Min(float64(rpmDesMagnitudeLinX), kLimitSpeedMaxRpm)

	// var rpmDesMagnitudeLinY = math.Abs(linear.Y / kWheelCircumferenceMm * 60)
	// rpmDesMagnitudeLinY = math.Min(float64(rpmDesMagnitudeLinY), kLimitSpeedMaxRpm)

	// var rpmDesMagnitudeAngZ = math.Abs(angular.Z / 360 * 60 * kWheelRevPerVehicleRev)
	// rpmDesMagnitudeAngZ = math.Min(rpmDesMagnitudeAngZ, kLimitSpeedMaxRpm)

	// var linearXNormal = math.Min(rpmDesMagnitudeLinX/kLimitSpeedMaxRpm, 1)
	// var linearYNormal = math.Min(rpmDesMagnitudeLinY/kLimitSpeedMaxRpm, 1)
	// var angularZNormal = math.Min(rpmDesMagnitudeAngZ/kLimitSpeedMaxRpm, 1)

	// var linearMagnitude = math.Sqrt(math.Pow(linearXNormal, 2) + math.Pow(linearYNormal, 2))
	// var linearAngle = math.Atan2(linear.Y, linear.X)
	// var rpmDesFr = math.Min(math.Sin(linearAngle-math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude + angularZNormal
	// var rpmDesFl = math.Min(math.Sin(linearAngle+math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude - angularZNormal
	// var rpmDesRr = math.Min(math.Sin(linearAngle+math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude + angularZNormal
	// var rpmDesRl = math.Min(math.Sin(linearAngle-math.Pi/4)*math.Sqrt(2), 1)*linearMagnitude - angularZNormal
	// var rpmMax = math.Max(math.Max(rpmDesFr, rpmDesFl), math.Max(rpmDesRr, rpmDesRl))

	// if rpmMax > 1 {
	// 	rpmDesFr /= rpmMax
	// 	rpmDesFl /= rpmMax
	// 	rpmDesRr /= rpmMax
	// 	rpmDesRl /= rpmMax
	// }

	// var baseCmd = mecanumCommand{
	// 	state:   mecanumStates[mecanumStateEnable],
	// 	mode:    mecanumModes[mecanumModeSpeed],
	// 	current: kDefaultCurrent,
	// 	encoder: 0,
	// }
	// var frCmd, flCmd, rrCmd, rlCmd = baseCmd, baseCmd, baseCmd, baseCmd
	// frCmd.rpm = int16(rpmDesFr * kLimitSpeedMaxRpm)
	// flCmd.rpm = int16(rpmDesFl * kLimitSpeedMaxRpm)
	// rrCmd.rpm = int16(rpmDesRr * kLimitSpeedMaxRpm)
	// rlCmd.rpm = int16(rpmDesRl * kLimitSpeedMaxRpm)

	// var canFrame = (&frCmd).toFrame(base.logger, kCanIdMotorFr)
	// if _, err := base.canTxSocket.Send(canFrame); err != nil {
	// 	base.logger.Errorw("spin command TX error", "error", err)
	// }

	// canFrame = (&flCmd).toFrame(base.logger, kCanIdMotorFl)
	// if _, err := base.canTxSocket.Send(canFrame); err != nil {
	// 	base.logger.Errorw("spin command TX error", "error", err)
	// }

	// canFrame = (&rrCmd).toFrame(base.logger, kCanIdMotorRr)
	// if _, err := base.canTxSocket.Send(canFrame); err != nil {
	// 	base.logger.Errorw("spin command TX error", "error", err)
	// }

	// canFrame = (&rlCmd).toFrame(base.logger, kCanIdMotorRl)
	// if _, err := base.canTxSocket.Send(canFrame); err != nil {
	// 	base.logger.Errorw("spin command TX error", "error", err)
	// }

	return nil
}

// Stop stops the base. It is assumed the base stops immediately.
func (base *intermodeOmniBase) Stop(ctx context.Context, extra map[string]interface{}) error {
	base.isMoving.Store(false)
	return base.setNextCommand(ctx, &stopCmd)
}

// DoCommand executes additional commands beyond the Base{} interface. For this rover that includes door open and close commands.
func (base *intermodeOmniBase) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
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
					telemSet(telemGearDesired, v)
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

func (i intermodeOmniBase) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return i.geometries, nil
}

func (i *intermodeOmniBase) Name() resource.Name {
	return resource.Name {
		API:    	base.API,
		Remote:	 	"test",
		Name:		"modal",
	}
}

func (i *intermodeOmniBase) Reconfigure(context.Context, resource.Dependencies, resource.Config) error {
	return nil
}

func (i *intermodeOmniBase) Properties(ctx context.Context, extra map[string]interface{}) (base.Properties, error) {
	return base.Properties {
		WidthMeters:				kVehicleTrackwidthMm/1000.0,
		WheelCircumferenceMeters:  	kWheelCircumferenceMm/1000.0,
	}, nil
}

func (base *intermodeOmniBase) IsMoving(ctx context.Context) (bool, error) {
	return base.isMoving.Load(), nil
}

// Close cleanly closes the base.
func (base *intermodeOmniBase) Close(ctx context.Context) error {
	base.setNextCommand(ctx, &stopCmd)
	base.cancel()
	base.activeBackgroundWorkers.Wait()

	return nil
}
