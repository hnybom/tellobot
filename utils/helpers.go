package utils

import (
	"fmt"
	"gocv.io/x/gocv"
)

func CapPower(power float32, maxPower float32) float32 {
	if (power > maxPower) {
		power = maxPower
	}
	return power
}

func Abs(powery float32) float32 {
	if (powery < 0) {
		powery = -1 * powery
	}
	return powery
}

func PrintIfChanged(val *int, bar *gocv.Trackbar, title string) {
	if *val != bar.GetPos() {
		fmt.Println(title, " : ", bar.GetPos())
		*val = bar.GetPos()
	}
}