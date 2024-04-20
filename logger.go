package ghetty

import (
	"fmt"
	"math"
	"os"
	"strconv"

	"github.com/klauspost/cpuid/v2"
)

type Logger struct {
	File *os.File

	CurrentFPS float64

	ShouldWrite bool
}

func NewLogger(directory string) Logger {
	var path string = directory + "/" + cpuid.CPU.BrandName + "/Scene" + strconv.Itoa(Scene)

	switch AlgorithmUsed {
	case BarycentricAlgorithm:
		path += "/Barycentric"
	case SweepLineAlgorithm:
		path += "/SweepLine"
	case EdgeTestAlgorithm:
		path += "/EdgeTest"
	}

	if err := os.MkdirAll(path, os.ModePerm); err != nil {
		panic(err)
	}

	path += "/" + strconv.Itoa(Cores) + ".txt"

	file, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0666)
	if err != nil {
		panic(err)
	}

	return Logger{file, 0, false}
}

func (logger *Logger) Log(framerate float64) {
	if math.Floor(framerate) > 0 {
		if logger.CurrentFPS != framerate {
			logger.CurrentFPS = framerate
			logger.File.WriteString(fmt.Sprintln(framerate))
		}
	}

}

func (logger *Logger) Close() {
	logger.File.Close()
}
