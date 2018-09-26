package drone

import (
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/keyboard"
	"image"
	"image/color"

	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
)

type DroneType int

const (
	DroneFake DroneType = iota
	DroneReal
)

type Drone interface {

	// Start starts the driver.
	Init() error

	// Halt stops the driver.
	Halt() (err error)

	// TakeOff tells drones to liftoff and start flying.
	TakeOff() (err error)

	// Throw & Go support
	ThrowTakeOff() (err error)

	// Land tells drone to come in for landing.
	Land() (err error)

	// StopLanding tells drone to stop landing.
	StopLanding() (err error)

	// PalmLand tells drone to come in for a hand landing.
	PalmLand() (err error)

	// SetExposure sets the drone camera exposure level. Valid levels are 0, 1, and 2.
	SetExposure(level int) (err error)

	// SetVideoEncoderRate sets the drone video encoder rate.
	SetVideoEncoderRate(rate tello.VideoBitRate) (err error)

	// SetFastMode sets the drone throttle to 1.
	SetFastMode() error

	// SetSlowMode sets the drone throttle to 0.
	SetSlowMode() error

	// Rate queries the current video bit rate.
	Rate() (err error)

	// Up tells the drone to ascend. Pass in an int from 0-100.
	Up(val int) error

	// Down tells the drone to descend. Pass in an int from 0-100.
	Down(val int) error

	// Forward tells the drone to go forward. Pass in an int from 0-100.
	Forward(val int) error

	// Backward tells drone to go in reverse. Pass in an int from 0-100.
	Backward(val int) error

	// Right tells drone to go right. Pass in an int from 0-100.
	Right(val int) error

	// Left tells drone to go left. Pass in an int from 0-100.
	Left(val int) error

	// Clockwise tells drone to rotate in a clockwise direction. Pass in an int from 0-100.
	Clockwise(val int) error

	// CounterClockwise tells drone to rotate in a counter-clockwise direction.
	// Pass in an int from 0-100.
	CounterClockwise(val int) error

	// Hover tells the drone to stop moving on the X, Y, and Z axes and stay in place
	Hover()

	// CeaseRotation stops any rotational motion
	CeaseRotation()

	// Bounce tells drone to start/stop performing the bouncing action
	Bounce() (err error)

	// Flip tells drone to flip
	Flip(direction tello.FlipType) (err error)

	// FrontFlip tells the drone to perform a front flip.
	FrontFlip() (err error)

	// BackFlip tells the drone to perform a back flip.
	BackFlip() (err error)

	// RightFlip tells the drone to perform a flip to the right.
	RightFlip() (err error)

	// LeftFlip tells the drone to perform a flip to the left.
	LeftFlip() (err error)

	// ParseFlightData from drone
	ParseFlightData(b []byte) (fd *tello.FlightData, err error)

	// GetVelocity gives the currently active speed in four axis
	// x-axis is velocity in right direction with values from -1.0 to 1.0
	// y-axis is velocity in up direction with values from -1.0 to 1.0
	// z-axis is velocity in forward direction with values from -1.0 to 1.0
	// w-axis is velocity in clockwise direction with values from -1.0 to 1.0
	GetVelocity() mgl32.Vec4

	// ReadVideoFrame reads the next frame from video stream
	ReadVideoFrame(frame *gocv.Mat) error

	// CameraMatrix returns the camera matrix
	CameraMatrix() *gocv.Mat

	// DistortionCoefficients returns the camera optics distortions
	DistortionCoefficients() *gocv.Mat

	// CameraToDrone returns the rotation matrix from camera coordinate system
	// to drone coordinate system
	CameraToDroneMatrix() mgl32.Mat3

	// DroneToCamera returns the rotation matrix from drone coordinate system
	// to camera coordinate system
	DroneToCameraMatrix() mgl32.Mat3
}

type handleKey func(event keyboard.KeyEvent, drone Drone)

func New(droneType DroneType, fn handleKey, cameraCalibrationFilename string) Drone {

	keys := keyboard.NewDriver()
	var d Drone

	switch droneType {
	case DroneFake:
		dt := &fakeDriver{}
		dt.cameraCalibrationFilename = cameraCalibrationFilename
		d = dt
	case DroneReal:
		dt := &realDriver{
			Driver: *tello.NewDriver("8890"),
		}
		dt.cameraCalibrationFilename = cameraCalibrationFilename
		d = dt
	}

	keybot := gobot.NewRobot("keyboard",
		[]gobot.Connection{},
		[]gobot.Device{keys},
		func() {},
	)

	keys.On(keyboard.Key, func(data interface{}) {
		key := data.(keyboard.KeyEvent)
		fn(key, d)
	})

	keybot.Start(false)

	return d
}

func DrawCrosshair(d Drone, img *gocv.Mat) {

	s := float32(img.Rows())
	center := mgl32.Vec3{0, 0, 1}
	// transform
	center = d.DroneToCameraMatrix().Mul3x1(center)

	pts := []mgl32.Vec3{
		center,
	}
	pts2d := contrib.ProjectPoints(pts, mgl32.Vec3{}, mgl32.Vec3{}, d.CameraMatrix(), d.DistortionCoefficients())

	x := int(pts2d[0][0])
	y := int(pts2d[0][1])

	cs := int(s * 0.05)

	x0 := image.Pt(x-cs, y)
	x1 := image.Pt(x+cs, y)
	y0 := image.Pt(x, y-cs)
	y1 := image.Pt(x, y+cs)

	gocv.Line(img, x0, x1, color.RGBA{0, 0, 0, 0}, 2)
	gocv.Line(img, y0, y1, color.RGBA{0, 0, 0, 0}, 2)

	gocv.Rectangle(img, image.Rect(x-cs/2, y-cs/2, x+cs/2, y+cs/2), color.RGBA{0, 0, 0, 0}, 2)
}

func DrawControls(d Drone, img *gocv.Mat) {

	width := img.Cols()
	height := img.Rows()

	s := float32(img.Rows())

	t := int(s * 0.25)
	a := int(s * 0.20)

	// axis
	gocv.Line(img, image.Pt(width-t-a, height-t), image.Pt(width-t+a, height-t), color.RGBA{0, 0, 0, 0}, 1)
	gocv.Line(img, image.Pt(width-t, height-t-a), image.Pt(width-t, height-t+a), color.RGBA{0, 0, 0, 0}, 1)

	gocv.Line(img, image.Pt(t-a, height-t), image.Pt(t+a, height-t), color.RGBA{0, 0, 0, 0}, 1)
	gocv.Line(img, image.Pt(t, height-t-a), image.Pt(t, height-t+a), color.RGBA{0, 0, 0, 0}, 1)

	v := d.GetVelocity()
	x := int(v[0] * float32(a))
	y := int(v[1] * float32(a))
	z := int(v[2] * float32(a))
	w := float64(v[3] * 180)
	// rotation
	gocv.Ellipse(img, image.Pt(width-t, height-t), image.Pt(a, a/2), 0, 270, 270+w, color.RGBA{255, 255, 0, 0}, 4)
	// left-right
	gocv.Line(img, image.Pt(width-t, height-t), image.Pt(width-t+x, height-t), color.RGBA{0, 255, 0, 0}, 2)
	// up-down
	gocv.Line(img, image.Pt(width-t, height-t), image.Pt(width-t, height-t+y), color.RGBA{0, 0, 255, 0}, 2)
	// forward-backward
	gocv.Line(img, image.Pt(t, height-t), image.Pt(t, height-t+z), color.RGBA{255, 0, 0, 0}, 2)

}
