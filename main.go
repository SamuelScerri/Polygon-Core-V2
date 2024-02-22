package main

import (
	"bufio"
	"fmt"
	"os"

	"github.com/veandco/go-sdl2/sdl"
)

var Cores = 1

const Width, Height = 640, 360
const FOV = 90

const Near, Far = .1, 1000
const Aspect = 16 / 9

var TileXSize, TileYSize = Width, Height

func BasicShader(s, t, w float32) (r, g, b float32) {
	r = s
	g = t
	b = w

	return
}

func main() {
	fmt.Println("Please Select Triangle Rasterization Algorithm:")

	fmt.Println("")

	fmt.Println("1: Sweep-Line Algorithm")
	fmt.Println("2: Barycentric Algorithm")
	fmt.Println("3: Edge Test Algorithm")

	fmt.Println("")

	reader := bufio.NewReader(os.Stdin)
	algorithm, _, _ := reader.ReadRune()

	var i int

	switch algorithm {
	case '1':
		AlgorithmUsed = SweepLineAlgorithm
	case '2':
		AlgorithmUsed = BarycentricAlgorithm
	case '3':
		AlgorithmUsed = EdgeTestAlgorithm
	}

	fmt.Println("")

	fmt.Print("Please Type How Many Cores Will Be Utilized: ")

	fmt.Scan(&Cores)

	fmt.Print("Please Type How Cores Will Be Divided Per Row: ")

	fmt.Scan(&i)
	TileXSize /= i

	fmt.Print("Please Type How Cores Will Be Divided Per Column: ")

	fmt.Scan(&i)
	TileYSize /= i

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

	var tiles [4][3]Tile

	for y := range tiles[0] {
		for x := range tiles {

			tiles[x][y].X = x * TileXSize
			tiles[x][y].Y = y * TileYSize
			tiles[x][y].Frame = surface.Pixels()
		}
	}

	Pitch = int(surface.Pitch)
	BytesPerPixel = surface.BytesPerPixel()

	var projectionMatrix Matrix = ProjectionMatrix()
	var position Vertex = Vertex{0, 0, 0, 0}

	running := true

	for running {
		//benchmark := time.Now()

		for event := sdl.PollEvent(); event != nil; event = sdl.PollEvent() {
			switch event.(type) {
			case *sdl.QuitEvent:
				println("Quit")
				running = false
			}
		}

		var state []uint8 = sdl.GetKeyboardState()

		if state[sdl.SCANCODE_W] == 1 {
			position[Z] -= 0.0078125
		}

		if state[sdl.SCANCODE_S] == 1 {
			position[Z] += 0.0078125
		}

		if state[sdl.SCANCODE_LEFT] == 1 {
			position[X] -= 0.001953125
		}

		if state[sdl.SCANCODE_RIGHT] == 1 {
			position[X] += 0.001953125
		}

		if state[sdl.SCANCODE_UP] == 1 {
			position[Y] -= 0.001953125
		}

		if state[sdl.SCANCODE_DOWN] == 1 {
			position[Y] += 0.001953125
		}

		var triangle Triangle = Triangle{
			Vertices: [3]Vertex{
				{0, -.5, 0, 1},
				{-.5, .5, 0, 1},
				{.5, .5, 0, 1},
			},
		}

		triangle.Shader = BasicShader

		var matrix Matrix = TransformationMatrix(position, Vertex{0, 0, 0, 1})

		triangle.Transform(&matrix)
		triangle.Transform(&projectionMatrix)

		Process(&triangle, &tiles)

		WaitGroup.Add(Cores)

		for y := range tiles[0] {
			for x := range tiles {
				go func(x, y int) {
					tiles[x][y].Clear(byte(x+y)*4, byte(x+y)*4, byte(x+y)*4)
					tiles[x][y].Rasterize(nil)

					WaitGroup.Done()
				}(x, y)
			}
		}

		WaitGroup.Wait()

		window.UpdateSurface()
	}
}
