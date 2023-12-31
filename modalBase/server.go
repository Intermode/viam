// Package main is an example of a custom viam server.

package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/edaniels/golog"
	"github.com/go-daq/canbus"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	goutils "go.viam.com/utils"
	viamutils "go.viam.com/utils"
	"golang.org/x/sys/unix"

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
	goutils.ContextualMain(mainWithArgs, golog.NewDevelopmentLogger("intermodeBaseModule"))
}

// /////////////
// Telemetry //
// /////////////
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

func telemGet(key string) interface{} {
	telemetryLock.RLock()
	defer telemetryLock.RUnlock()
	return telemetry[key]
}

func telemGetAll() map[string]interface{} {
	telemetryLock.RLock()
	defer telemetryLock.RUnlock()
	toReturn := map[string]interface{}{
		telemGearDesired:   byte(0),
		telemSpeed:         math.NaN(),
		telemSpeedLimit:    float64(-1),
		telemSteerAngle:    -1,
		telemStateOfCharge: -1,
	}
	for k, v := range telemetry {
		toReturn[k] = v
	}
	return toReturn
}

// /////////////
// Fail-Safe //
// /////////////
const commsTimeoutIntervalMs = 1000 // If it has been at least this long since last command received, execute containment
var commsTimeout time.Time
var commsTimeoutEnable = true		// Enable or disable comms timeout
									// Presently changed based off of received command style

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

	err = socketRecv.SetFilters([]unix.CanFilter{
		{Id: telemDriveId, Mask: unix.CAN_SFF_MASK},
		{Id: telemWheelSpeedId, Mask: unix.CAN_SFF_MASK},
		{Id: telemBatteryPowerId, Mask: unix.CAN_SFF_MASK},
		{Id: telemBatteryStateId, Mask: unix.CAN_SFF_MASK},
	})
	if err != nil {
		return nil, err
	}

	if err := socketRecv.Bind(channel); err != nil {
		return nil, err
	}

	cancelCtx, cancel := context.WithCancel(context.Background())
	iBase := &interModeBase{
		name:          name,
		headLightsOn:  true,
		nextCommandCh: make(chan canbus.Frame),
		cancel:        cancel,
		logger:        logger,
	}
	iBase.isMoving.Store(false)

	iBase.activeBackgroundWorkers.Add(2)
	viamutils.ManagedGo(func() {
		publishThread(cancelCtx, *socketSend, iBase.nextCommandCh, logger)
	}, iBase.activeBackgroundWorkers.Done)
	viamutils.ManagedGo(func() {
		receiveThread(cancelCtx, *socketRecv, logger)
	}, iBase.activeBackgroundWorkers.Done)

	return iBase, nil
}

// constants from the data sheet.
const (
	channel        = "can0"
	driveId uint32 = 0x220
	lightId uint32 = 0x260

	// Telemetry
	telemDriveId        uint32 = 0x240
	telemWheelSpeedId   uint32 = 0x241
	telemBatteryPowerId uint32 = 0x250
	telemBatteryStateId uint32 = 0x251
)

const (
	gearPark          = "park"
	gearReverse       = "reverse"
	gearNeutral       = "neutral"
	gearDrive         = "drive"
	gearEmergencyStop = "emergency_stop"

	steerModeFrontWheelDrive = "front-wheel-drive"
	steerModeRearWheelDrive  = "rear-wheel-drive"
	steerModeFourWheelDrive  = "four-wheel-drive"
	steerModeCrabSteering    = "crab-steering"

	rightTurnSignal = "right-turn-signal"
	leftTurnSignal  = "left-turn-signal"
	hazards         = "hazards"
	headLights      = "head-lights"

	frontDoor = "front-door"
	rearDoor  = "rear-door"

	telemGearDesired   = "desired_gear"
	telemSpeed         = "speed"
	telemSpeedLimit    = "speed_limit"
	telemSteerAngle    = "steer_angle"
	telemStateOfCharge = "state_of_charge"

	kNumBitsPerByte = 8
)

var (
	gears = map[string]byte{
		gearPark:          0,
		gearReverse:       1,
		gearNeutral:       2,
		gearDrive:         3,
		gearEmergencyStop: 4,
	}
	steerModes = map[string]byte{
		steerModeFrontWheelDrive: 0,
		steerModeRearWheelDrive:  1,
		steerModeFourWheelDrive:  2,
		steerModeCrabSteering:    3,
	}
	lightBits = map[string]byte{
		// the spec sheet states the right and left swap, but this appeared backwards on my modal
		// couldn't find a place where my logic flips these, but could be wrong.
		rightTurnSignal: 0x1,
		leftTurnSignal:  0x2,
		hazards:         0x4,
		headLights:      0x8,
	}
	doorID = map[string]uint32{
		frontDoor: 0x270,
		rearDoor:  0x280,
	}
)

type doorCommand struct {
	DoorID uint32
	Open   bool
}

type lightCommand struct {
	RightTurnSignal bool
	LeftTurnSignal  bool
	Hazards         bool
	HeadLights      bool
}

type driveCommand struct {
	Accelerator   float64
	Brake         float64
	SteeringAngle float64
	Gear          byte
	SteerMode     byte
}

// calculateSteeringAngleBytes returns the intermode specific angle bytes for the given angle.
func calculateSteeringAngleBytes(angle float64) []byte {
	// angle from -90 to 90
	// positive is left, negative is right
	if math.Abs(angle) > 90 {
		if math.Signbit(angle) {
			angle = -90
		} else {
			angle = 90
		}
	}

	value := int16(angle / 0.0078125) // intermode scalar

	angleBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(angleBytes, uint16(value))
	return angleBytes
}

// calculateAccelAndBrakeBytes returns the intermode specific acceleration and brake bytes for the given
// acceleration percentage.
func calculateAccelAndBrakeBytes(accelPct float64, brakePct float64) []byte {
	var speedLimit = SPEED_LIMP_HOME
	var ok = false
	{
		// Apply speed (throttle) limit
		speedLimit, ok = telemGet(telemSpeedLimit).(float64)
	}
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

type modalCommand interface {
	toFrame(logger golog.Logger) canbus.Frame
}

// toFrame convert the light command to a canbus data frame.
func (cmd *doorCommand) toFrame(logger golog.Logger) canbus.Frame {
	frame := canbus.Frame{
		ID:   cmd.DoorID,
		Data: make([]byte, 0, 8),
		Kind: canbus.SFF,
	}

	var cmdByte byte
	if cmd.Open {
		cmdByte = 0x1
	} else {
		cmdByte = 0x0
	}

	frame.Data = append(frame.Data, cmdByte)
	logger.Debugw("frame", "data", frame.Data)

	return frame
}

// toFrame convert the light command to a canbus data frame.
func (cmd *lightCommand) toFrame(logger golog.Logger) canbus.Frame {
	frame := canbus.Frame{
		ID:   lightId,
		Data: make([]byte, 0, 8),
		Kind: canbus.SFF,
	}

	var cmdByte byte = 0x0

	if cmd.RightTurnSignal {
		cmdByte |= lightBits[rightTurnSignal]
	}
	if cmd.LeftTurnSignal {
		cmdByte |= lightBits[leftTurnSignal]
	}
	if cmd.Hazards {
		cmdByte |= lightBits[hazards]
	}
	if cmd.HeadLights {
		cmdByte |= lightBits[headLights]
	}

	frame.Data = append(frame.Data, cmdByte)
	logger.Debugw("frame", "data", frame.Data)

	return frame
}

// toFrame convert the drive command to a canbus data frame.
func (cmd *driveCommand) toFrame(logger golog.Logger) canbus.Frame {
	frame := canbus.Frame{
		ID:   driveId,
		Data: make([]byte, 0, 8),
		Kind: canbus.SFF,
	}
	frame.Data = append(frame.Data, calculateAccelAndBrakeBytes(cmd.Accelerator, cmd.Brake)...)
	frame.Data = append(frame.Data, calculateSteeringAngleBytes(cmd.SteeringAngle)...)
	// is this the best place to be setting the gear to reverse? felt better than in each place that sets the forward motion.
	if cmd.Accelerator < 0 {
		cmd.Gear = gears[gearReverse]
	}
	frame.Data = append(frame.Data, cmd.Gear, cmd.SteerMode)

	logger.Debugw("frame", "data", frame.Data)

	return frame
}

type interModeBase struct {
	name                    string
	headLightsOn            bool
	isMoving                atomic.Bool
	nextCommandCh           chan canbus.Frame
	activeBackgroundWorkers sync.WaitGroup
	cancel                  func()
	logger                  golog.Logger
}

// publishThread continuously sends the current drive command over the canbus.
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
		case frame = <-nextCommandCh:
			if frame.ID == driveId {
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
			logger.Errorw("drive command send error", "error", err)
		}
	}
}

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
func receiveThread(
	ctx context.Context,
	socket canbus.Socket,
	logger golog.Logger,
) {
	defer socket.Close()

	for {
		if ctx.Err() != nil {
			return
		}

		frame, err := socket.Recv()
		if err != nil {
			logger.Errorw("CAN Rx error", "error", err)
		}

		switch frame.ID {
		case telemDriveId:
			// Apply speed (throttle) limit
			telemSet(telemSteerAngle, fCanExtractSignal(frame.Data, canSignalDriveSteerAngle))
		case telemWheelSpeedId:
			var speedFl = fCanExtractSignal(frame.Data, canSignalWheelSpeedFrontLeft)
			var speedFr = fCanExtractSignal(frame.Data, canSignalWheelSpeedFrontRight)
			var speedRl = fCanExtractSignal(frame.Data, canSignalWheelSpeedRearLeft)
			var speedRr = fCanExtractSignal(frame.Data, canSignalWheelSpeedRearRight)
			telemSet(telemSpeed, (speedFl+speedFr+speedRl+speedRr)/4)
		case telemBatteryStateId:
			telemSet(telemStateOfCharge, fCanExtractSignal(frame.Data, canSignalBatteryStateOfCharge))
		}
	}
}

/*
	InterMode Base Implementation
	Every method will set the next command for the publish loop to send over the command bus forever.
*/

// MoveStraight moves the base forward the given distance and speed.
func (base *interModeBase) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	cmd := driveCommand{
		Accelerator:   0.5,
		Brake:         0,
		SteeringAngle: 0,
		Gear:          gears[gearDrive],
		SteerMode:     steerModes[steerModeFourWheelDrive],
	}

	if mmPerSec < 0 || distanceMm < 0 {
		cmd.Accelerator *= -1
	}

	if err := base.setNextCommand(ctx, &cmd); err != nil {
		return err
	}
	base.isMoving.Store(true)

	defer func() {
		base.setNextCommand(ctx, &stopCmd)
		base.isMoving.Store(false)
	}()

	if !viamutils.SelectContextOrWait(ctx, time.Duration(mmPerSec/float64(distanceMm))) {
		return ctx.Err()
	}

	return nil
}

// Spin spins the base by the given angleDeg and degsPerSec.
func (base *interModeBase) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	//TODO: make headlights and hazard persistent parts of the base
	if err := base.setNextCommand(ctx, &lightCommand{
		RightTurnSignal: angleDeg < 0,
		LeftTurnSignal:  angleDeg > 0,
		Hazards:         false,
		HeadLights:      base.headLightsOn,
	}); err != nil {
		return err
	}

	if err := base.setNextCommand(ctx, &driveCommand{
		Accelerator:   0.5,
		Brake:         0,
		SteeringAngle: angleDeg,
		Gear:          gears[gearDrive],
		SteerMode:     steerModes[steerModeFourWheelDrive],
	}); err != nil {
		return err
	}
	base.isMoving.Store(true)

	defer func() {
		base.setNextCommand(ctx, &stopCmd)
		base.isMoving.Store(false)
	}()

	if !viamutils.SelectContextOrWait(ctx, time.Duration(angleDeg/math.Abs(degsPerSec))) {
		return ctx.Err()
	}

	return base.setNextCommand(ctx, &lightCommand{
		RightTurnSignal: false,
		LeftTurnSignal:  false,
		Hazards:         false,
		HeadLights:      base.headLightsOn,
	})
}

// SetPower sets the linear and angular [-1, 1] drive power.
func (base *interModeBase) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	//TODO: make headlights and hazard persistent parts of the base
	if err := base.setNextCommand(ctx, &lightCommand{
		RightTurnSignal: angular.Z < -0.3,
		LeftTurnSignal:  angular.Z > 0.3,
		Hazards:         false,
		HeadLights:      base.headLightsOn,
	}); err != nil {
		return err
	}

	var accel float64 = 0
	var brake float64 = 0
	var ok = false
	var gearDesired byte = 0x0
	{
		// If the desired gear got corrupted, default to emergency stop.
		gearDesired, ok = telemGet(telemGearDesired).(byte)
		if !ok {
			gearDesired = gears[gearEmergencyStop]
		}

		accel = linear.Y
		brake = linear.X

		base.isMoving.Store(telemGet(telemSpeed) != 0)
	}
	return base.setNextCommand(ctx, &driveCommand{
		Accelerator:   accel,
		Brake:         brake,
		SteeringAngle: angular.Z * STEERANGLE_MAX,
		Gear:          gearDesired,
		SteerMode:     steerModes[steerModeFourWheelDrive],
	})
}

// SetVelocity sets the linear (mmPerSec) and angular (degsPerSec) velocity.
func (base *interModeBase) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	//TODO: make headlights and hazard persistent parts of the base
	if err := base.setNextCommand(ctx, &lightCommand{
		RightTurnSignal: angular.Z < -30,
		LeftTurnSignal:  angular.Z > 30,
		Hazards:         false,
		HeadLights:      base.headLightsOn,
	}); err != nil {
		return err
	}

	// if any component of either vector isnt 0, we're moving!
	base.isMoving.Store(linear.X != 0 || linear.Y != 0 || linear.Z != 0 || angular.X != 0 || angular.Y != 0 || angular.Z != 0)
	return base.setNextCommand(ctx, &driveCommand{
		Accelerator:   linear.Y,
		Brake:         0,
		SteeringAngle: angular.Z * STEERANGLE_MAX,
		Gear:          gears[gearDrive],
		SteerMode:     steerModes[steerModeFourWheelDrive],
	})
}

var stopCmd = driveCommand{
	Accelerator:   0,
	Brake:         1,
	SteeringAngle: 0,
	Gear:          gears[gearPark],
	SteerMode:     steerModes[steerModeFourWheelDrive],
}

var emergencyCmd = driveCommand{
	Accelerator:   0,
	Brake:         1,
	SteeringAngle: 0,
	Gear:          gears[gearEmergencyStop],
	SteerMode:     steerModes[steerModeFourWheelDrive],
}

// Stop stops the base. It is assumed the base stops immediately.
func (base *interModeBase) Stop(ctx context.Context, extra map[string]interface{}) error {
	base.isMoving.Store(false)
	return base.setNextCommand(ctx, &stopCmd)
}

// DoCommand executes additional commands beyond the Base{} interface. For this rover that includes door open and close commands.
func (base *interModeBase) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// TODO: expand this function to change steering/gearing modes.
	name, ok := cmd["command"]
	if !ok {
		return nil, errors.New("missing 'command' value")
	}
	switch name {
	case "set_door":
		doorRaw, ok := cmd["door"]
		if !ok {
			return nil, errors.New("door must be set, one of front-door|rear-door")
		}
		door, ok := doorRaw.(string)
		if !ok {
			return nil, errors.New("door value must be a string")
		}
		door = strings.ToLower(door)
		if !(door == frontDoor || door == rearDoor) {
			return nil, errors.New("door value must be one of front-door|rear-door")
		}
		openRaw, ok := cmd["open"]
		if !ok {
			return nil, errors.New("open must be set and a boolean value")
		}
		open, ok := openRaw.(bool)
		if !ok {
			return nil, errors.New("open value must be a boolean")
		}

		cmd := doorCommand{DoorID: doorID[door], Open: open}
		if err := base.setNextCommand(ctx, &cmd); err != nil {
			return nil, err
		}

		return map[string]interface{}{"return": "set_door command processed"}, nil

	case "set_headlights":
		onRaw, ok := cmd["on"]
		if !ok {
			return nil, errors.New("on must be set and a boolean value")
		}
		on, ok := onRaw.(bool)
		if !ok {
			return nil, errors.New("on value must be a boolean")
		}

		base.headLightsOn = on
		cmd := lightCommand{
			RightTurnSignal: false,
			LeftTurnSignal:  false,
			Hazards:         false,
			HeadLights:      base.headLightsOn}
		if err := base.setNextCommand(ctx, &cmd); err != nil {
			return nil, err
		}

		return map[string]interface{}{"return": "set_headlights command processed"}, nil

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

	case "set_speed_limit":
		speedLimitRaw, ok := cmd["speed_limit"]
		if !ok {
			return nil, errors.New("speed_limit must be set to a float")
		}
		speedLimit, ok := speedLimitRaw.(float64)
		if !ok {
			return nil, errors.New("speed_limit value must be a float")
		}
		telemSet(telemSpeedLimit, speedLimit)
		return map[string]interface{}{"return": fmt.Sprintf("set_speed_limit command processed: %f", speedLimit)}, nil

	case "get_telemetry":
		return map[string]interface{}(telemGetAll()), nil

	default:
		return nil, fmt.Errorf("no such command: %s", name)
	}
}

func (base *interModeBase) IsMoving(ctx context.Context) (bool, error) {
	return base.isMoving.Load(), nil
}

// Close cleanly closes the base.
func (base *interModeBase) Close() {
	base.cancel()
	base.activeBackgroundWorkers.Wait()
}

func (base *interModeBase) setNextCommand(ctx context.Context, cmd modalCommand) error {
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
