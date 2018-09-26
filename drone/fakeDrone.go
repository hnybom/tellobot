package drone

import (
	"fmt"

	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
)

type fakeDriver struct {
	velocity                  mgl32.Vec4
	webcam                    *gocv.VideoCapture
	cameraCalibrationFilename string
	camMatrix                 gocv.Mat
	distCoeffs                gocv.Mat
	cameraToDrone             mgl32.Mat3
}

func (d *fakeDriver) Init() error {
	d.cameraToDrone = mgl32.Rotate3DX(mgl32.DegToRad(-13.0))
	var err error
	d.webcam, err = gocv.VideoCaptureDevice(0)
	if err != nil {
		return err
	}

	d.camMatrix, d.distCoeffs = contrib.ReadCameraParameters(d.cameraCalibrationFilename)

	return nil
}
func (d *fakeDriver) Halt() (err error) {
	return nil
}

func (d *fakeDriver) TakeOff() (err error) {
	fmt.Println("drone: TakeOff")
	return nil
}

func (d *fakeDriver) ThrowTakeOff() (err error) {
	return nil
}

func (d *fakeDriver) Land() (err error) {
	fmt.Println("drone: Land")
	return nil
}

func (d *fakeDriver) StopLanding() (err error) {
	return nil
}

func (d *fakeDriver) PalmLand() (err error) {
	return nil
}

func (d *fakeDriver) SetExposure(level int) (err error) {
	return nil
}

func (d *fakeDriver) SetVideoEncoderRate(rate tello.VideoBitRate) (err error) {
	return nil
}

func (d *fakeDriver) SetFastMode() error {
	return nil
}

func (d *fakeDriver) SetSlowMode() error {
	return nil
}

func (d *fakeDriver) Rate() (err error) {
	return nil
}

func (d *fakeDriver) Right(val int) error {
	d.velocity[0] = float32(val) / 100.0
	return nil
}

func (d *fakeDriver) Left(val int) error {
	d.velocity[0] = float32(val) / 100.0 * -1
	return nil
}

func (d *fakeDriver) Up(val int) error {
	d.velocity[1] = float32(val) / 100.0
	return nil
}

func (d *fakeDriver) Down(val int) error {
	d.velocity[1] = float32(val) / 100.0 * -1
	return nil
}

func (d *fakeDriver) Forward(val int) error {
	d.velocity[2] = float32(val) / 100.0
	return nil
}

func (d *fakeDriver) Backward(val int) error {
	d.velocity[2] = float32(val) / 100.0 * -1
	return nil
}

func (d *fakeDriver) Clockwise(val int) error {
	d.velocity[3] = float32(val) / 100.0
	return nil
}

func (d *fakeDriver) CounterClockwise(val int) error {
	d.velocity[3] = float32(val) / 100.0 * -1
	return nil
}

func (d *fakeDriver) Hover() {
	d.velocity[0] = 0.0
	d.velocity[1] = 0.0
	d.velocity[2] = 0.0
}
func (d *fakeDriver) CeaseRotation() {
	d.velocity[3] = 0.0
}

func (d *fakeDriver) Bounce() (err error) {
	return nil
}

func (d *fakeDriver) Flip(direction tello.FlipType) (err error) {
	return nil
}

func (d *fakeDriver) FrontFlip() (err error) {
	return nil
}

func (d *fakeDriver) BackFlip() (err error) {
	return nil
}

func (d *fakeDriver) RightFlip() (err error) {
	return nil
}

func (d *fakeDriver) LeftFlip() (err error) {
	return nil
}

func (d *fakeDriver) ParseFlightData(b []byte) (fd *tello.FlightData, err error) {
	return nil, nil
}

func (d *fakeDriver) GetVelocity() mgl32.Vec4 {
	return d.velocity
}
func (d *fakeDriver) ReadVideoFrame(frame *gocv.Mat) error {
	d.webcam.Read(frame)
	return nil
}

func (d *fakeDriver) CameraMatrix() *gocv.Mat {
	return &d.camMatrix
}

func (d *fakeDriver) DistortionCoefficients() *gocv.Mat {
	return &d.distCoeffs
}

func (d *fakeDriver) CameraToDroneMatrix() mgl32.Mat3 {
	return d.cameraToDrone
}

func (d *fakeDriver) DroneToCameraMatrix() mgl32.Mat3 {
	return d.cameraToDrone.Inv()
}
