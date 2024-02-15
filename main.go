package main

import (
	"github.com/veandco/go-sdl2/sdl"
)

const Cores = 12
const Width, Height = 640, 360
const FOV = 90

const Near, Far = .1, 1000
const Aspect = 16 / 9

const TileXSize, TileYSize = Width / 4, Height / 3

func BasicShader(s, t, w float32) (r, g, b float32) {
	r = s
	g = t
	b = w

	return
}

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

	var projectionMatrix Matrix = ProjectionMatrix()
	var position Vertex = Vertex{0, 0, -20, 0}

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

		var state []uint8 = sdl.GetKeyboardState()

		if state[sdl.SCANCODE_W] == 1 {
			position[Z] -= .0078125
		}

		if state[sdl.SCANCODE_S] == 1 {
			position[Z] += .0078125
		}

		if state[sdl.SCANCODE_LEFT] == 1 {
			position[X] -= .001953125
		}

		if state[sdl.SCANCODE_RIGHT] == 1 {
			position[X] += .001953125
		}

		if state[sdl.SCANCODE_UP] == 1 {
			position[Y] -= .001953125
		}

		if state[sdl.SCANCODE_DOWN] == 1 {
			position[Y] += .001953125
		}

		var triangle Triangle = Triangle{
			Vertices: [3]Vertex{
				{0, -.5, 0, 1},
				{-.5, .5, 0, 1},
				{.5, .75, 0, 1},
			},
		}

		var matrix Matrix = TransformationMatrix(position, Vertex{0, 0, 0, 0})

		triangle.Transform(&matrix)
		triangle.Transform(&projectionMatrix)

		frameBuffer.Clear(16, 16, 16)
		frameBuffer.Rasterize(&triangle, BasicShader)

		window.UpdateSurface()

		//fmt.Println(1 / float32(time.Since(benchmark).Seconds()))
	}
}
