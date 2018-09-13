package main

import (
	"fmt"
	"gocv.io/x/gocv"
	"image"
	"image/color"
	"tellobot/tracking"
)

var (
	hmaxTracking, hminTracking, sminTracking, smaxTracking, vminTracking,vmaxTracking *gocv.Trackbar
	hmax, hmin, smin,smax,vmin,vmax int
)



func main() {

	deviceID := 0

	trackingBar := gocv.NewWindow("Tracking sliders")
	hmaxTracking = trackingBar.CreateTrackbar("H_MAX", 255) // 200
	hmaxTracking.SetPos(224)
	hminTracking = trackingBar.CreateTrackbar("H_MIN", 255) // 0
	hminTracking.SetPos(163)
	sminTracking = trackingBar.CreateTrackbar("S_MIN", 255) // 180
	sminTracking.SetPos(115)
	smaxTracking = trackingBar.CreateTrackbar("S_MAX", 255) // 240
	smaxTracking.SetPos(255)
	vminTracking = trackingBar.CreateTrackbar("V_MIN", 255) // 170
	vminTracking.SetPos(50)
	vmaxTracking = trackingBar.CreateTrackbar("V_MAX", 255) // 210
	vmaxTracking.SetPos(242)

	trackingBar.ResizeWindow(800, 600)
	defer trackingBar.Close()

	webcam, err := gocv.VideoCaptureDevice(int(deviceID))
	if err != nil {
		fmt.Println(err)
		return
	}
	defer webcam.Close()

	// open display window
	window := gocv.NewWindow("Face Detect")
	defer window.Close()

	targetWindow := gocv.NewWindow("Target")

	// prepare image matrix
	img := gocv.NewMat()
	targetImg := gocv.NewMat()

	defer targetImg.Close()
	defer img.Close()

	fmt.Printf("start reading camera device: %v\n", deviceID)
	for {
		if ok := webcam.Read(&img); !ok {
			fmt.Printf("cannot read device %d\n", deviceID)
			return
		}
		if img.Empty() {
			continue
		}

		targetImg = img.Clone()

		filterImage(img, &img)
		printIfChanged(&hmax, hmaxTracking.GetPos(), "HMAX")
		printIfChanged(&hmin, hminTracking.GetPos(), "HMIN")
		printIfChanged(&smin, sminTracking.GetPos(), "SMIN")
		printIfChanged(&smax, smaxTracking.GetPos(), "SMAX")
		printIfChanged(&vmin, vminTracking.GetPos(), "VMIN")
		printIfChanged(&vmax, vmaxTracking.GetPos(), "VMAX")

		x := 0
		y := 0
		distance := float32(0)

		tracking.TrackFilteredObject(&x, &y, &distance, img)

		maxY := targetImg.Rows()
		maxX := targetImg.Cols()

		window.IMShow(img)
		if window.WaitKey(1) >= 0 {
			break
		}

		if trackingBar.WaitKey(1) >= 0 {
			break
		}

		gocv.Circle(&targetImg, image.Point{x,y}, 10, color.RGBA{0, 0, 255, 0}, 2)

		targetY := maxY / 2
		targetX := maxX / 2

		xdiff := x - targetX
		powerx := (xdiff * 100) / maxX

		if(x != 0 && y != 0) {

			if(powerx > 10 || powerx < -10) {
				if(xdiff > 0) {
					fmt.Println("Power to RIGHT with ", powerx)
				} else {
					fmt.Println("Power to LEFT with ", powerx)
				}
			}

			ydiff := y - targetY
			powery := (ydiff * 100) / maxY

			if(powery > 10 || powery < -10) {
				if(ydiff > 0) {
					fmt.Println("Power to DOWN with ", powery)
				} else {
					fmt.Println("Power to UP with ", powery)
				}
			}
		}

		gocv.Circle(&targetImg, image.Point{targetX,targetY}, 10, color.RGBA{0, 255, 0, 0}, 2)

		targetWindow.IMShow(targetImg)
		if targetWindow.WaitKey(1) >= 0 {
			break
		}


	}
}

func printIfChanged(val *int, otherVal int, title string) {
	if(*val != otherVal) {
		fmt.Println(title, " : ", otherVal)
		*val = otherVal
	}
}

func filterImage(img gocv.Mat, dest *gocv.Mat) {
	gocv.CvtColor(img, dest, gocv.ColorBGRToHSV)

	gocv.InRangeWithScalar(*dest,
		gocv.NewScalar(
			float64(hminTracking.GetPos()),
			float64(sminTracking.GetPos()),
			float64(vminTracking.GetPos()),
			0),
		gocv.NewScalar(
			float64(hmaxTracking.GetPos()),
			float64(smaxTracking.GetPos()),
			float64(vmaxTracking.GetPos()),
			0),
		dest)

	morphOps(dest)


}

func morphOps(thresh *gocv.Mat) {

	erodeElement := gocv.GetStructuringElement(gocv.MorphRect, image.Point{3, 3 })
	dilateElement := gocv.GetStructuringElement(gocv.MorphRect, image.Point{8, 8 })

	gocv.Erode(*thresh, thresh, erodeElement);
	gocv.Erode(*thresh, thresh, erodeElement);
	gocv.Dilate(*thresh, thresh, dilateElement);
	gocv.Dilate(*thresh, thresh, dilateElement);

}