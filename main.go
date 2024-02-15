package main

import (
	"github.com/veandco/go-sdl2/sdl"
)

const Cores = 12
const Width, Height = 640, 360
const TileXSize, TileYSize = Width / 4, Height / 3

func main() {
	if err := sdl.Init(sdl.INIT_EVERYTHING); err != nil {
		panic(err)
	}
	defer sdl.Quit()

	window, err := sdl.CreateWindow("Ghetty Engine - V2", sdl.WINDOWPOS_CENTERED, sdl.WINDOWPOS_CENTERED,
		Width, Height, sdl.WINDOW_SHOWN)
	if err != nil {
		panic(err)
	}
	defer window.Destroy()

	surface, err := window.GetSurface()
	if err != nil {
		panic(err)
	}

	var frameBuffer Buffer = Buffer{Frame: surface.Pixels(), Depth: make([]float32, Width*Height), Pitch: int(surface.Pitch), BytesPerPixel: surface.BytesPerPixel()}

	running := true
	for running {
		//benchmark := time.Now()

		for event := sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
			switch event.(type) {
			case *sdl.QuitEvent:
				println("Quit")
				running = false
				break
			}
		}

		var triangle Triangle = Triangle{
			Vertices: [3]Vertex{
				{0, -.5, 0},
				{-.5, .5, 0},
				{.5, .75, 0},
			},
		}

		frameBuffer.Clear(16, 16, 16)
		frameBuffer.Rasterize(&triangle)

		window.UpdateSurface()

		//fmt.Println(1 / float32(time.Since(benchmark).Seconds()))
	}
}
