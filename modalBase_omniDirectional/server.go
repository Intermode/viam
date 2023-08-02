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
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/resource"
)

// Modal limits
const PEDAL_MAX = 100.0
const STEERANGLE_MAX = 25.0
const SPEED_LIMP_HOME = 20.0 // Max speed (throttle) if a limp home condition is active

var model = resource.NewModel("rdk", "builtin", "intermode")

func main() {
	goutils.ContextualMain(mainWithArgs, golog.NewDevelopmentLogger("intermodeOmniBaseModule"))
}

// // /////////////
// // Telemetry //
// // /////////////
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

// func telemGet(key string) interface{} {
// 	telemetryLock.RLock()
// 	defer telemetryLock.RUnlock()
// 	return telemetry[key]
// }

// func telemGetAll() map[string]interface{} {
// 	telemetryLock.RLock()
// 	defer telemetryLock.RUnlock()
// 	toReturn := map[string]interface{}{
// 		telemGearDesired:   byte(0),
// 		telemSpeed:         math.NaN(),
// 		telemSpeedLimit:    float64(-1),
// 		telemSteerAngle:    -1,
// 		telemStateOfCharge: -1,
// 	}
// 	for k, v := range telemetry {
// 		toReturn[k] = v
// 	}
// 	return toReturn
// }

// // /////////////
// // Fail-Safe //
// // /////////////
// const commsTimeoutIntervalMs = 1000 // If it has been at least this long since last command received, execute containment
// var commsTimeout time.Time
// var commsTimeoutEnable = true		// Enable or disable comms timeout
// 									// Presently changed based off of received command style

const (
	// channel         = "can0"
	channel = "vcan0"

	// Vehicle properties
	kVehicleWheelbaseMm  = 709.684
	kVehicleTrackwidthMm = 528.580
	kVehicleSeparation   = (kVehicleWheelbaseMm + kVehicleTrackwidthMm) / 2

	// Wheel properties
	kWheelRadiusMm        float64 = 76.2
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

	telemGearDesired   = "desired_gear"
	telemSpeed         = "speed"
	telemSpeedLimit    = "speed_limit"
	telemSteerAngle    = "steer_angle"
	telemStateOfCharge = "state_of_charge"

	mecanumStateDisable   = "disable"
	mecanumStateEnable    = "enable"
	mecanumStateResetErr  = "resetError"
	mecanumStateResetPos  = "resetPosition"
	mecanumStateResetCal  = "resetCalibration"
	mecanumStateCalSensor = "calibrateSensor"

	mecanumModeSpeed    = "speed"
	mecanumModeAbsolute = "absolute"
	mecanumModeRelative = "relative"
	mecanumModeCurrent  = "current"

	kNumBitsPerByte = 8

	kCanIdMotorFr uint32 = 0x0000022A
	kCanIdMotorFl uint32 = 0x0000022B
	kCanIdMotorRr uint32 = 0x0000022C
	kCanIdMotorRl uint32 = 0x0000022D

	kCanIdTelemWheelSpeedId   uint32 = 0x241
	kCanIdTelemBatteryPowerId uint32 = 0x250
	kCanIdTelemBatteryStateId uint32 = 0x251
)

var (
	mecanumStates = map[string]byte{
		mecanumStateDisable:   0x00,
		mecanumStateEnable:    0x01,
		mecanumStateResetErr:  0x02,
		mecanumStateResetPos:  0x03,
		mecanumStateResetCal:  0x04,
		mecanumStateCalSensor: 0x05,
	}
	mecanumModes = map[string]byte{
		mecanumModeSpeed:    0x00,
		mecanumModeAbsolute: 0x01,
		mecanumModeRelative: 0x02,
		mecanumModeCurrent:  0x03,
	}

	// Constants that must be calculated at runtime
	// Distance wheel tirepatch is from center of vehicle
	kVehicleTirePatchRadiusMm = math.Sqrt(math.Pow(kVehicleWheelbaseMm/2, 2) + math.Pow(kVehicleTrackwidthMm/2, 2))
	// Circumference of circle with radius of tirepatch distance from center of vehicle
	kVehicleTirePatchCircumferenceMm = 2 * math.Pi * kVehicleTirePatchRadiusMm

	// Wheel revolutions to vehicle rotations
	kWheelRevPerVehicleRev = kVehicleTirePatchCircumferenceMm / kWheelCircumferenceMm

	// Max magnitude of a command in the x-y plane
	kMaxLinearMagnitude = math.Sqrt(math.Pow(kMagnitudeMaxX, 2) + math.Pow(kMagnitudeMaxX, 2))
)

type mecanumCommand struct {
	state   byte
	mode    byte
	rpm     int16
	current int16
	encoder int32
}

type intermodeOmniBase struct {
	name                    string
	canTxSocket             canbus.Socket
	isMoving                atomic.Bool
	activeBackgroundWorkers sync.WaitGroup
	cancel                  func()
	logger                  golog.Logger
}

type modalCommand interface {
	toFrame(logger golog.Logger) canbus.Frame
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) (err error) {
	registerBase()
	modalModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}
	modalModule.AddModelFromRegistry(ctx, base.Subtype, model)

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
	registry.RegisterComponent(
		base.Subtype,
		model,
		registry.Component{Constructor: func(
			ctx context.Context,
			deps registry.Dependencies,
			config config.Component,
			logger golog.Logger,
		) (interface{}, error) {
			return newBase(config.Name, logger)
		}})
}

// newBase creates a new base that underneath the hood sends canbus frames via
// a 10ms publishing loop.
func newBase(name string, logger golog.Logger) (base.Base, error) {
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

	// err = socketRecv.SetFilters([]unix.CanFilter{
	// 	{Id: kCanIdTelemWheelSpeedId, Mask: unix.CAN_SFF_MASK},
	// 	{Id: kCanIdTelemBatteryPowerId, Mask: unix.CAN_SFF_MASK},
	// 	{Id: kCanIdTelemBatteryStateId, Mask: unix.CAN_SFF_MASK},
	// })
	// if err != nil {
	// 	return nil, err
	// }

	if err := socketRecv.Bind(channel); err != nil {
		return nil, err
	}

	_, cancel := context.WithCancel(context.Background())
	iBase := &intermodeOmniBase{
		name:        name,
		canTxSocket: *socketSend,
		cancel:      cancel,
		logger:      logger,
	}
	iBase.isMoving.Store(false)

	// iBase.activeBackgroundWorkers.Add(2)
	// viamutils.ManagedGo(func() {
	// 	publishThread(cancelCtx, *socketSend, iBase.nextCommandCh, logger)
	// }, iBase.activeBackgroundWorkers.Done)
	// viamutils.ManagedGo(func() {
	// 	receiveThread(cancelCtx, *socketRecv, logger)
	// }, iBase.activeBackgroundWorkers.Done)

	return iBase, nil
}

/*
 * Convert the a mecanum command to a CAN frame
 */
func (cmd *mecanumCommand) toFrame(logger golog.Logger, canId uint32) canbus.Frame {
	frame := canbus.Frame{
		ID:   canId,
		Data: make([]byte, 0, 8),
		Kind: canbus.EFF,
	}
	frame.Data = append(frame.Data, (cmd.state&0x0F)|((cmd.mode&0x0F)<<4))
	frame.Data = append(frame.Data, byte(cmd.rpm&0xFF))
	frame.Data = append(frame.Data, byte((cmd.rpm>>8)&0xFF)|byte((cmd.current&0x0F)<<4))
	frame.Data = append(frame.Data, byte((cmd.current>>4)&0x0FF))
	frame.Data = append(frame.Data, byte(cmd.encoder&0xFF))
	frame.Data = append(frame.Data, byte((cmd.encoder>>8)&0xFF))
	frame.Data = append(frame.Data, byte((cmd.encoder>>16)&0xFF))
	frame.Data = append(frame.Data, byte((cmd.encoder>>24)&0xFF))

	logger.Debugw("frame", "data", frame.Data)

	return frame
}

// // publishThread continuously sends the current drive command over the canbus.
// func publishThread(
// 	ctx context.Context,
// 	socket canbus.Socket,
// 	nextCommandCh chan canbus.Frame,
// 	logger golog.Logger,
// ) {
// 	defer socket.Close()
// 	commsTimeout = time.Now().Add(commsTimeoutIntervalMs * time.Millisecond)

// 	driveFrame := (&stopCmd).toFrame(logger)
// 	var frame canbus.Frame

// 	for {
// 		if ctx.Err() != nil {
// 			return
// 		}
// 		select {
// 		case <-ctx.Done():
// 		case <-time.After(10 * time.Millisecond):
// 		}
// 		if commsTimeoutEnable && time.Now().After(commsTimeout) {
// 			driveFrame = (&emergencyCmd).toFrame(logger)
// 		}
// 		if _, err := socket.Send(driveFrame); err != nil {
// 			logger.Errorw("drive command send error", "error", err)
// 		}
// 	}
// }

/*
 *	The CAN RX support functions were translated from C++ by ChatGPT, so might
 * 		be weird and not stylistically consistent
 */
func ucGetByteBitMask(byteNum uint8, bitSigLsb uint8, bitSigMsb uint8) uint8 {
	/*
	 * Returns a bit mask for a single byte of a CAN payload
	 *
	 * Inputs:
	 * 	`byteNum` : array byte number to make a mask for
	 * 	`bitSigLsb` : least significant bit for the signal (in the CAN payload)
	 * 	`bitSigMsb` : most significant bit for the signal (in the CAN payload)
	 */
	var bitMaskLsb uint8 = 0 // Least significant bit for the bit mask
	var bitMaskMsb uint8 = 0 // Most significant bit for the bit mask
	var bitByteLsb int32 = 0 // Least significant bit for the current byte
	var bitByteMsb int32 = 0 // Most significant bit for the current byte

	bitByteLsb = int32(byteNum) * kNumBitsPerByte
	bitByteMsb = (int32(byteNum)+1)*kNumBitsPerByte - 1
	if int32(bitSigLsb) < bitByteLsb {
		bitMaskLsb = 0
	} else {
		bitMaskLsb = uint8(int32(bitSigLsb) - bitByteLsb)
	}

	if int32(bitSigMsb) >= bitByteMsb {
		bitMaskMsb = kNumBitsPerByte - 1
	} else {
		bitMaskMsb = uint8(int32(bitSigMsb) - bitByteLsb)
	}

	return uint8((math.MaxUint8 << (bitMaskMsb + 1)) ^ (math.MaxUint8 << bitMaskLsb))
}

/*
 * CAN signal definition
 */
type CanSignal_TypeDef struct {
	fScalar     float32 // Scalar
	fOffset     float32 // Offset
	ucStart     uint8   // Start bit
	ucLength    uint8   // Length in bits
	ucLittleEnd uint8   // Endianness
	ucSigned    uint8   // Signed
}

// Constructor
func NewCanSignal_TypeDef(fScalar, fOffset float32, ucStart, ucLength, ucLittleEnd, ucSigned uint8) *CanSignal_TypeDef {
	return &CanSignal_TypeDef{
		fScalar:     fScalar,
		fOffset:     fOffset,
		ucStart:     ucStart,
		ucLength:    ucLength,
		ucLittleEnd: ucLittleEnd,
		ucSigned:    ucSigned,
	}
}

/*
 * Extracts a CAN signal
 *
 * Currently limited to 32-bit signals to avoid casting the entire data
 *  array to a 64 bit number to simplify switch to CAN FD (allows up to 64
 *  data bytes)
 *
 * Caller to cast the signal result to an alternate data type if needed
 *
 * Inputs:
 * 	`pucData` : CAN data array
 * 	`xSignal` : signal configuration data
 */
func fCanExtractSignal(pucData []uint8, xSignal CanSignal_TypeDef) float32 {
	var ulRet uint32 = 0      // Return value
	var fRet float32 = 0      // Float-casted return value
	var ucByteShift uint8 = 0 // Number of bytes to shift single signal bytes

	// Least significant signal bit
	var ucBitSigLsb uint8 = xSignal.ucStart
	// Most significant signal bit
	//  Reduced by one to account for zero indexing of bits
	var ucBitSigMsb uint8 = uint8(ucBitSigLsb + xSignal.ucLength - 1)
	var ucByteStart uint8 = ucBitSigLsb / kNumBitsPerByte // Start byte
	var ucByteStop uint8 = ucBitSigMsb / kNumBitsPerByte  // Stop byte

	for i := ucByteStart; i <= ucByteStop; i++ {
		if 0 < xSignal.ucLittleEnd {
			ucByteShift = uint8(i - ucByteStart)
		} else { // Big Endian
			ucByteShift = uint8(ucByteStop - i)
		}

		ulRet += (uint32(ucGetByteBitMask(i, ucBitSigLsb, ucBitSigMsb)) & uint32(pucData[i])) <<
			(ucByteShift * kNumBitsPerByte)
	}

	ulRet >>= ucBitSigLsb - kNumBitsPerByte*ucByteStart

	// If signed, extend the sign bit
	if 0 < xSignal.ucSigned {
		if int32(ulRet)&(1<<(ucBitSigMsb-ucByteStart*kNumBitsPerByte)) != 0 {
			ulRet |= uint32(math.MaxUint32) <<
				(ucBitSigMsb - ucByteStart*kNumBitsPerByte + 1)
		}
		fRet = float32(int32(ulRet))
	} else {
		fRet = float32(ulRet)
	}

	return fRet*xSignal.fScalar + xSignal.fOffset
}

var (
	canSignalDriveSteerAngle      = *NewCanSignal_TypeDef(0.0078125, 0, 32, 16, 1, 1)
	canSignalWheelSpeedFrontLeft  = *NewCanSignal_TypeDef(0.0078125, 0, 0, 16, 1, 1)
	canSignalWheelSpeedFrontRight = *NewCanSignal_TypeDef(0.0078125, 0, 16, 16, 1, 1)
	canSignalWheelSpeedRearLeft   = *NewCanSignal_TypeDef(0.0078125, 0, 32, 16, 1, 1)
	canSignalWheelSpeedRearRight  = *NewCanSignal_TypeDef(0.0078125, 0, 48, 16, 1, 1)
	canSignalBatteryStateOfCharge = *NewCanSignal_TypeDef(0.1, 0, 0, 16, 1, 0)
)

// receiveThread receives canbus frames and stores data when necessary.
// func receiveThread(
// 	ctx context.Context,
// 	socket canbus.Socket,
// 	logger golog.Logger,
// ) {
// 	defer socket.Close()

// 	for {
// 		if ctx.Err() != nil {
// 			return
// 		}

// 		frame, err := socket.Recv()
// 		if err != nil {
// 			logger.Errorw("CAN Rx error", "error", err)
// 		}

// 		switch frame.ID {
// 		case kCanIdTelemWheelSpeedId:
// 			var speedFl = fCanExtractSignal(frame.Data, canSignalWheelSpeedFrontLeft)
// 			var speedFr = fCanExtractSignal(frame.Data, canSignalWheelSpeedFrontRight)
// 			var speedRl = fCanExtractSignal(frame.Data, canSignalWheelSpeedRearLeft)
// 			var speedRr = fCanExtractSignal(frame.Data, canSignalWheelSpeedRearRight)
// 			telemSet(telemSpeed, (speedFl+speedFr+speedRl+speedRr)/4)
// 		case kCanIdTelemBatteryStateId:
// 			telemSet(telemStateOfCharge, fCanExtractSignal(frame.Data, canSignalBatteryStateOfCharge))
// 		}
// 	}
// }

/*
	InterMode Base Implementation
	Every method will set the next command for the publish loop to send over the command bus forever.
*/

// MoveStraight moves the base forward the given distance and speed.
// TODO: Move with the correct units
func (base *intermodeOmniBase) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Speed
	var speedNegative = mmPerSec < 0
	var rpsDes = mmPerSec / kWheelCircumferenceMm
	var rpmDesMagnitude = int16(math.Abs(rpsDes * 60))
	rpmDesMagnitude = int16(math.Min(float64(rpmDesMagnitude), kLimitSpeedMaxRpm))

	// Distance
	var distanceNegative = distanceMm < 0
	var distanceRev = float64(distanceMm) / kWheelCircumferenceMm
	var encoderMagnitude = math.Abs(float64(kWheelTicksPerRev) * distanceRev)
	// Add 0.5 for rounding purposes
	var encoderValue = int32(encoderMagnitude + 0.5)

	if true == distanceNegative || true == speedNegative {
		encoderValue *= -1
	}

	// TODO: Actually rotate degrees
	var cmd = mecanumCommand{
		state:   mecanumStates[mecanumStateEnable],
		mode:    mecanumModes[mecanumModeRelative],
		rpm:     rpmDesMagnitude,
		current: kDefaultCurrent,
		encoder: encoderValue,
	}

	var canFrame = (&cmd).toFrame(base.logger, kCanIdMotorFr)
	base.logger.Debugw("frame", "data", canFrame.Data)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("straight command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorRr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("straight command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorFl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("straight command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorRl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("straight command TX error", "error", err)
	}

	if !viamutils.SelectContextOrWait(ctx, time.Duration(distanceMm/int(math.Abs(mmPerSec)))) {
		return ctx.Err()
	}

	return nil
}

// Spin spins the base by the given angleDeg and degsPerSec.
// TODO: Move with the correct units
func (base *intermodeOmniBase) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Speed
	var speedNegative = degsPerSec < 0
	var rpmDesMagnitude = math.Abs(degsPerSec / 360 * 60 * kWheelRevPerVehicleRev)
	rpmDesMagnitude = math.Min(rpmDesMagnitude, kLimitSpeedMaxRpm)

	// Angle
	var angleNegative = angleDeg < 0
	var encoderMagnitude = math.Abs(angleDeg/360*kWheelRevPerVehicleRev) * float64(kWheelTicksPerRev)
	var encoderValue = int32(encoderMagnitude)

	if true == speedNegative || true == angleNegative {
		encoderValue *= -1
	}

	var rightCmd = mecanumCommand{
		state:   mecanumStates[mecanumStateEnable],
		mode:    mecanumModes[mecanumModeRelative],
		rpm:     int16(rpmDesMagnitude),
		current: kDefaultCurrent,
		encoder: encoderValue,
	}
	var leftCmd = mecanumCommand{
		state:   mecanumStates[mecanumStateEnable],
		mode:    mecanumModes[mecanumModeRelative],
		rpm:     int16(rpmDesMagnitude),
		current: kDefaultCurrent,
		encoder: -1 * encoderValue,
	}

	var canFrame = (&rightCmd).toFrame(base.logger, kCanIdMotorFr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&rightCmd).toFrame(base.logger, kCanIdMotorRr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&leftCmd).toFrame(base.logger, kCanIdMotorFl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&leftCmd).toFrame(base.logger, kCanIdMotorRl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	if !viamutils.SelectContextOrWait(ctx, time.Duration(angleDeg/math.Abs(degsPerSec))) {
		return ctx.Err()
	}

	return nil
}

// SetPower sets the linear and angular [-1, 1] drive power.
func (base *intermodeOmniBase) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Some vector components do not apply to a 2D base
	if 0 != linear.Z {
		base.logger.Warnw("Linear Z command non-zero and has no effect")
	}
	if 0 != angular.X {
		base.logger.Warnw("Angular X command non-zero and has no effect")
	}
	if 0 != angular.Y {
		base.logger.Warnw("Angular Y command non-zero and has no effect")
	}

	var linearMagnitude = math.Sqrt(math.Pow(linear.X, 2) + math.Pow(linear.Y, 2)) / kMaxLinearMagnitude
	var linearAngle = math.Atan2(linear.Y, linear.X)
	// Angular multiplied by 0.5 because that is the max single-linear-direction magnitude
	var rpmDesFr = math.Sin(linearAngle-math.Pi/4)*linearMagnitude + angular.Z*0.5
	var rpmDesFl = math.Sin(linearAngle+math.Pi/4)*linearMagnitude - angular.Z*0.5
	var rpmDesRr = math.Sin(linearAngle+math.Pi/4)*linearMagnitude + angular.Z*0.5
	var rpmDesRl = math.Sin(linearAngle-math.Pi/4)*linearMagnitude - angular.Z*0.5
	var rpmMax = math.Max(math.Max(rpmDesFr, rpmDesFl), math.Max(rpmDesRr, rpmDesRl))

	if rpmMax > 1 {
		rpmDesFr /= rpmMax
		rpmDesFl /= rpmMax
		rpmDesRr /= rpmMax
		rpmDesRl /= rpmMax
	}

	var baseCmd = mecanumCommand{
		state:   mecanumStates[mecanumStateEnable],
		mode:    mecanumModes[mecanumModeSpeed],
		current: kDefaultCurrent,
		encoder: 0,
	}
	var frCmd, flCmd, rrCmd, rlCmd = baseCmd, baseCmd, baseCmd, baseCmd
	
	frCmd.rpm = int16(rpmDesFr*kLimitSpeedMaxRpm)
	flCmd.rpm = int16(rpmDesFl*kLimitSpeedMaxRpm)
	rrCmd.rpm = int16(rpmDesRr*kLimitSpeedMaxRpm)
	rlCmd.rpm = int16(rpmDesRl*kLimitSpeedMaxRpm)

	var canFrame = (&frCmd).toFrame(base.logger, kCanIdMotorFr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&flCmd).toFrame(base.logger, kCanIdMotorFl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&rrCmd).toFrame(base.logger, kCanIdMotorRr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&rlCmd).toFrame(base.logger, kCanIdMotorRl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	return nil
}

// SetVelocity sets the linear (mmPerSec) and angular (degsPerSec) velocity.
func (base *intermodeOmniBase) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	base.isMoving.Store(true) // TODO: Replace with feedback info

	// Some vector components do not apply to a 2D base
	if 0 != linear.Z {
		base.logger.Warnw("Linear Z command non-zero and has no effect")
	}
	if 0 != angular.X {
		base.logger.Warnw("Angular X command non-zero and has no effect")
	}
	if 0 != angular.Y {
		base.logger.Warnw("Angular Y command non-zero and has no effect")
	}

	var rpmDesFr, rpmDesFl, rpmDesRr, rpmDesRl float64
	var rpmDesX = linear.X / kWheelCircumferenceMm * 60
	var rpmDesY = linear.Y / kWheelCircumferenceMm * 60
	var rpmDesSpin = angular.Z / 360 * 60 * kWheelRevPerVehicleRev

	rpmDesFr = (rpmDesX + rpmDesY + kVehicleSeparation*rpmDesSpin) / kWheelRadiusMm
	rpmDesFl = (rpmDesX - rpmDesY - kVehicleSeparation*rpmDesSpin) / kWheelRadiusMm
	rpmDesRr = (rpmDesX - rpmDesY + kVehicleSeparation*rpmDesSpin) / kWheelRadiusMm
	rpmDesRl = (rpmDesX + rpmDesY - kVehicleSeparation*rpmDesSpin) / kWheelRadiusMm

	var baseCmd = mecanumCommand{
		state:   mecanumStates[mecanumStateEnable],
		mode:    mecanumModes[mecanumModeSpeed],
		current: kDefaultCurrent,
		encoder: 0,
	}
	var frCmd, flCmd, rrCmd, rlCmd = baseCmd, baseCmd, baseCmd, baseCmd
	frCmd.rpm = int16(rpmDesFr)
	flCmd.rpm = int16(rpmDesFl)
	rrCmd.rpm = int16(rpmDesRr)
	rlCmd.rpm = int16(rpmDesRl)

	var canFrame = (&frCmd).toFrame(base.logger, kCanIdMotorFr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&flCmd).toFrame(base.logger, kCanIdMotorFl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&rrCmd).toFrame(base.logger, kCanIdMotorRr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	canFrame = (&rlCmd).toFrame(base.logger, kCanIdMotorRl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("spin command TX error", "error", err)
	}

	return nil
}

// Stop stops the base. It is assumed the base stops immediately.
func (base *intermodeOmniBase) Stop(ctx context.Context, extra map[string]interface{}) error {
	base.isMoving.Store(false) // TODO: Replace with feedback info

	var cmd = mecanumCommand{
		state:   mecanumStates[mecanumStateEnable],
		mode:    mecanumModes[mecanumModeSpeed],
		rpm:     0,
		current: kDefaultCurrent,
		encoder: 0,
	}

	var canFrame = (&cmd).toFrame(base.logger, kCanIdMotorFr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("stop command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorFl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("stop command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorRr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("stop command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorRl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("stop command TX error", "error", err)
	}

	return nil
}

// DoCommand executes additional commands beyond the Base{} interface. For this rover that includes door open and close commands.
func (base *intermodeOmniBase) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// TODO: expand this function to change steering/gearing modes.
	name, ok := cmd["command"]
	if !ok {
		return nil, errors.New("missing 'command' value")
	}
	switch name {
	default:
		return nil, fmt.Errorf("no such command: %s", name)
	}
}

func (base *intermodeOmniBase) IsMoving(ctx context.Context) (bool, error) {
	return base.isMoving.Load(), nil
}

// Close cleanly closes the base.
func (base *intermodeOmniBase) Close() {
	// Disable the wheels
	var cmd = mecanumCommand{
		state:   mecanumStates[mecanumStateDisable],
		mode:    mecanumModes[mecanumModeSpeed],
		rpm:     0,
		current: kDefaultCurrent,
		encoder: 0,
	}

	var canFrame = (&cmd).toFrame(base.logger, kCanIdMotorFr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("close command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorFl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("close command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorRr)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("close command TX error", "error", err)
	}

	canFrame = (&cmd).toFrame(base.logger, kCanIdMotorRl)
	if _, err := base.canTxSocket.Send(canFrame); err != nil {
		base.logger.Errorw("close command TX error", "error", err)
	}

	base.cancel()
	base.activeBackgroundWorkers.Wait()
}
