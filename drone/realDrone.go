package drone

import (
	"fmt"
	"io"
	"os/exec"
	"strconv"
	"time"

	"github.com/go-gl/mathgl/mgl32"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
)

type realDriver struct {
	tello.Driver
	velocity                  mgl32.Vec4
	cameraCalibrationFilename string
	camMatrix                 gocv.Mat
	distCoeffs                gocv.Mat
	frameBuf                  []byte
	ffmpegOut                 io.ReadCloser
	cameraToDrone             mgl32.Mat3
}

const (
	frameX    = 400
	frameY    = 300
	frameSize = frameX * frameY * 3
)

func (d *realDriver) Init() error {
	d.cameraToDrone = mgl32.Rotate3DX(mgl32.DegToRad(-13.0))

	// init ffmpeg
	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")

	ffmpegIn, _ := ffmpeg.StdinPipe()
	d.ffmpegOut, _ = ffmpeg.StdoutPipe()

	// init video buffer to hold the frame
	d.frameBuf = make([]byte, frameSize)

	d.camMatrix, d.distCoeffs = contrib.ReadCameraParameters(d.cameraCalibrationFilename)

	work := func() {
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		d.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			d.StartVideo()
			d.SetVideoEncoderRate(tello.VideoBitRate1M)
			//d.SetVideoEncoderRate(tello.VideoBitRate4M)
			d.SetExposure(1)

			gobot.Every(100*time.Millisecond, func() {
				d.StartVideo()
			})
		})

		d.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{d},
		work,
	)
	robot.Start(false)
	return nil
}

func (d *realDriver) Right(val int) error {
	d.velocity[0] = float32(val) / 100.0
	return d.Driver.Right(val)
}

func (d *realDriver) Left(val int) error {
	d.velocity[0] = float32(val) / 100.0 * -1
	return d.Driver.Left(val)
}

func (d *realDriver) Up(val int) error {
	d.velocity[1] = float32(val) / 100.0
	return d.Driver.Up(val)
}

func (d *realDriver) Down(val int) error {
	d.velocity[1] = float32(val) / 100.0 * -1
	return d.Driver.Down(val)
}

func (d *realDriver) Forward(val int) error {
	d.velocity[2] = float32(val) / 100.0
	return d.Driver.Forward(val)
}

func (d *realDriver) Backward(val int) error {
	d.velocity[2] = float32(val) / 100.0 * -1
	return d.Driver.Forward(val)
}

func (d *realDriver) Clockwise(val int) error {
	d.velocity[3] = float32(val) / 100.0
	return d.Driver.Clockwise(val)
}

func (d *realDriver) CounterClockwise(val int) error {
	d.velocity[3] = float32(val) / 100.0 * -1
	return d.Driver.CounterClockwise(val)
}

func (d *realDriver) Hover() {
	d.velocity[0] = 0.0
	d.velocity[1] = 0.0
	d.velocity[2] = 0.0
	d.Driver.Hover()
}
func (d *realDriver) CeaseRotation() {
	d.velocity[3] = 0.0
	d.Driver.CeaseRotation()
}

func (d *realDriver) GetVelocity() mgl32.Vec4 {
	return d.velocity
}

func (d *realDriver) ReadVideoFrame(frame *gocv.Mat) error {
	_, err := io.ReadFull(d.ffmpegOut, d.frameBuf)
	if err != nil {
		return err
	}
	*frame, err = gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, d.frameBuf)
	return nil
}
func (d *realDriver) CameraMatrix() *gocv.Mat {
	return &d.camMatrix
}

func (d *realDriver) DistortionCoefficients() *gocv.Mat {
	return &d.distCoeffs
}

func (d *realDriver) CameraToDroneMatrix() mgl32.Mat3 {
	return d.cameraToDrone
}

func (d *realDriver) DroneToCameraMatrix() mgl32.Mat3 {
	return d.cameraToDrone.Inv()
}
