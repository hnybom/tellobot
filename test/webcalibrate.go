package main

import (
	"gocv.io/x/gocv/contrib"
)

func main() {
	contrib.CalibrateCameraChArUco(12, 9, 0.30, 0.26)
}

