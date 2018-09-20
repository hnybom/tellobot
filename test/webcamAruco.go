package main

import (
	"fmt"
	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
	"image"
	"image/color"
	"tellobot/tracking"
)


func main() {

	deviceID := 0
	dict := contrib.NewArucoPredefinedDictionary(contrib.ArucoPredefinedDict_5x5_50)
	defer dict.Close()

	webcam, err := gocv.VideoCaptureDevice(int(deviceID))
	if err != nil {
		fmt.Println(err)
		return
	}
	defer webcam.Close()

	// open display window
	window := gocv.NewWindow("Face Detect")
	defer window.Close()

	// prepare image matrix
	img := gocv.NewMat()
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

		corners, ids := dict.DetectMarkers(&img)

		if len(corners) > 0 {
			dict.DrawDetectedMarkers(&img, corners, ids, color.RGBA{255, 0,0,0})

			x, y, _ := tracking.TrackAruco(corners, ids)
			gocv.Circle(&img, image.Point{x,y}, 10, color.RGBA{0, 0, 255, 0}, 2)
			maxY := img.Rows()
			maxX := img.Cols()
			flyTrackingWeb(x, y, maxX, maxY)
		}

		fmt.Println(corners)

		window.IMShow(img)

		if window.WaitKey(1) >= 0 {
			break
		}


	}
}

func flyTrackingWeb(x int, y int, maxX int, maxY int) {
	targetY := maxY / 2
	targetX := maxX / 2

	xdiff := x - targetX
	powerx := (xdiff * 200) / maxX

	ydiff := y - targetY
	powery := (ydiff * 200) / maxY

	if powerx > 10 || powerx < -10 {
		if xdiff > 0 {
			fmt.Println("Power to RIGHT with ", powerx)
		} else {
			fmt.Println("Power to LEFT with ", -1 * powerx)
		}
	}


	if powery > 10 || powery < -10 {
		if ydiff > 0 {
			fmt.Println("Power to DOWN with ", powery)
		} else {
			fmt.Println("Power to UP with ", -1 * powery)
		}
	}

}

