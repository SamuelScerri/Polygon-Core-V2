package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strconv"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

var Cores = 1

const Width, Height, Scale = 320, 180, 4
const FOV = 90

const Near, Far = .1, 1000
const Aspect = float32(Width) / float32(Height)

var TileXSize, TileYSize = Width, Height

var Tiles [][]Tile
var Buffer []byte = make([]byte, Width*Height*4)
var Depth []float32 = make([]float32, Width*Height)

var Brick Texture = LoadTexture("images/Cobble.png")
var Cobble Texture = LoadTexture("images/Brick.png")

var Triangles []Triangle

type Game struct{}

func BasicShader(w, s, t float32) (r, g, b float32) {
	r = w
	g = s
	b = t

	return
}

func WhiteShader(w, s, t float32) (r, g, b float32) {
	r = 1
	g = 1
	b = 1

	return
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return Width, Height
}

func (g *Game) Update() error {
	return nil
}

var Position Vertex = Vertex{0, 0, 15, 0}
var Projection Matrix = ProjectionMatrix()

func (g *Game) Draw(screen *ebiten.Image) {

	if ebiten.IsKeyPressed(ebiten.KeyRight) {
		Position[X] += .125 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyLeft) {
		Position[X] -= .125 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyDown) {
		Position[Y] += .125 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyUp) {
		Position[Y] -= .125 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyW) {
		Position[Z] += .125
	}

	if ebiten.IsKeyPressed(ebiten.KeyS) {
		Position[Z] -= .125
	}

	var matrix Matrix = TransformationMatrix(Position, Vertex{0, 0, 0, 1})
	matrix = matrix.Multiply(&Projection)

	var trianglesPerCore int = len(Triangles) / Cores

	WaitGroup.Add(Cores)

	for i := 0; i < Cores; i++ {
		go func(offset int) {
			for p := offset; p < offset+trianglesPerCore; p++ {
				var copiedTriangle Triangle = Triangles[p].Copy()
				copiedTriangle.Transform(&matrix)

				var processedTriangles []ProcessedTriangle = BuildAndProcess(&copiedTriangle)

				for index := range processedTriangles {
					var xMin, yMin, xMax, yMax int = processedTriangles[index].TileBoundary(&Tiles)

					for y := yMin; y < yMax; y++ {
						for x := xMin; x < xMax; x++ {
							Mutex.Lock()
							Tiles[x][y].Add(&processedTriangles[index])
							Mutex.Unlock()
						}
					}
				}
			}

			WaitGroup.Done()
		}(trianglesPerCore * i)
	}

	WaitGroup.Wait()

	WaitGroup.Add(Cores)

	for y := range Tiles[0] {
		for x := range Tiles {
			go func(x, y int) {
				Tiles[x][y].Clear(byte(x+y)*4, byte(x+y)*4, byte(x+y)*4)
				Tiles[x][y].Rasterize()

				WaitGroup.Done()
			}(x, y)
		}
	}

	WaitGroup.Wait()

	screen.WritePixels(Buffer)
	ebitenutil.DebugPrint(screen, "FPS: "+strconv.Itoa(int(ebiten.ActualFPS())))
}

func init() {
	fmt.Println("Loading Assets")
}

func main() {
	fmt.Println("1: Sweep-Line Algorithm")
	fmt.Println("2: Barycentric Algorithm")
	fmt.Println("3: Edge Test Algorithm")

	fmt.Print("\nPlease Select Triangle Rasterization Algorithm: ")

	reader := bufio.NewReader(os.Stdin)
	algorithm, _, _ := reader.ReadRune()

	switch algorithm {
	case '1':
		AlgorithmUsed = SweepLineAlgorithm
	case '2':
		AlgorithmUsed = BarycentricAlgorithm
	case '3':
		AlgorithmUsed = EdgeTestAlgorithm
	}

	fmt.Print("\nPlease Type How Many Cores Will Be Utilized: ")

	fmt.Scan(&Cores)

	var yTiles int = int(math.Floor(math.Sqrt(float64(Cores))))
	var xTiles int = Cores / yTiles

	TileXSize /= xTiles
	TileYSize /= yTiles

	Tiles = make([][]Tile, xTiles)

	for x := range Tiles {
		Tiles[x] = make([]Tile, yTiles)

		for y := range Tiles[0] {
			Tiles[x][y].X = x * TileXSize
			Tiles[x][y].Y = y * TileYSize
			Tiles[x][y].Frame = Buffer
			Tiles[x][y].Depth = Depth
		}
	}

	for i := 0; i < 1000; i++ {
		var triangle Triangle = Triangle{
			Vertices: [3]Vertex{
				{0, -.25, -1, 1},
				{.25, .25, 0, 1},
				{-.25, .5, 0, 1},
			},

			UV: [3]Vertex{
				{0.5, 1},
				{1, 0},
				{0, 0},
			},

			Color: [3]Vertex{
				{rand.Float32(), rand.Float32(), rand.Float32()},
				{rand.Float32(), rand.Float32(), rand.Float32()},
				{rand.Float32(), rand.Float32(), rand.Float32()},
			},
		}

		if math.Round(rand.Float64()) == 0 {
			triangle.Shader = BasicShader
			triangle.Texture = &Brick
		} else {
			triangle.Shader = WhiteShader
			triangle.Texture = &Cobble
		}

		var matrix Matrix = TransformationMatrix(Vertex{rand.Float32()*4 - 2, rand.Float32()*4 - 2, rand.Float32() * -30, 0}, Vertex{0, 0, 0, 0})
		triangle.Transform(&matrix)
		Triangles = append(Triangles, triangle)
	}

	ebiten.SetWindowSize(Width*Scale, Height*Scale)
	ebiten.SetWindowTitle("Ghetty Engine")
	ebiten.SetTPS(ebiten.SyncWithFPS)
	ebiten.SetScreenClearedEveryFrame(false)
	ebiten.SetVsyncEnabled(false)

	if err := ebiten.RunGame(&Game{}); err != nil {
		log.Fatal(err)
	}
}
