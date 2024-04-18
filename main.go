package ghetty

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

var Cores, Scene = 1, 1

const Width, Height, Scale = 160, 90, 4
const FOV = 150

const Near, Far = .1, 1000
const Aspect = float32(Width) / float32(Height)

var TileXSize, TileYSize = Width, Height
var Time float32

type Callback func()

var OnRender Callback

func BasicVertex(vertex, uv, normal, color *Vertex, matrices ...*Matrix) {
	//(*vertex)[X] += float32(math.Sin(float64(Time*.0125+(*vertex)[Y]))) * .25

	(*color)[R] = 1
	(*color)[G] = 1
	(*color)[B] = 1

	//var red Vertex = Vertex{1, 0, 0}

	//color.Interpolate(&red, (*vertex)[Y] * .001)

	for index := range matrices {
		vertex.Transform(matrices[index])
	}

	//(*vertex)[X] = float32(math.Round(float64((*vertex)[X]) * 32)) / 32;
	//(*vertex)[Y] = float32(math.Round(float64((*vertex)[Y]) * 32)) / 32;
	//(*vertex)[Z] = float32(math.Round(float64((*vertex)[Z]) * 32)) / 32;
}

func BasicFragment(r, g, b *float32, uv *Vertex, textures ...*Texture) {
	for index := range textures {
		var tr, tg, tb, _ float32 = textures[index].Get(textures[index].ConvertPosition(uv))
		*r, *g, *b = *r*tr, *g*tg, *b*tb
	}
}

var Basic Shader = Shader{
	BasicVertex, BasicFragment,
}

var Tiles [][]Tile
var Buffer []byte = make([]byte, Width*Height*4)
var Depth []float32 = make([]float32, Width*Height)

var Brick Texture = LoadTexture("images/Brick.png")
var Cobble Texture = LoadTexture("images/Brick.png")

var Bunny = LoadModel("models/Teapot.obj")

var Log Logger

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

var Position Vertex = Vertex{0, 0, 0, 0}
var Projection Matrix = ProjectionMatrix()

func (g *Game) Draw(screen *ebiten.Image) {
	if ebiten.IsKeyPressed(ebiten.KeyRight) {
		Position[X] -= .125 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyLeft) {
		Position[X] += .125 / 2 / 2
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

	var trianglesPerCore int = len(Triangles) / Cores
	var totalTrianglesRasterized int = 0

	Time += 1

	var matrix Matrix = TransformationMatrix(Position, Vertex{0, 0, 0, 1})
	matrix = matrix.Multiply(&Projection)

	WaitGroup.Add(Cores)

	for i := 0; i < Cores; i++ {
		go func(offset int) {
			for p := offset; p < offset+trianglesPerCore; p++ {
				var copiedTriangle Triangle = Triangles[p].Copy()

				for index := range copiedTriangle.Vertices {
					copiedTriangle.Shader.Vertex(&copiedTriangle.Vertices[index],
						&copiedTriangle.UV[index],
						&copiedTriangle.Normals[index],
						&copiedTriangle.Color[index],
						&matrix)
				}

				BuildAndProcess(&copiedTriangle, &Tiles)
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

	OnRender()

	screen.WritePixels(Buffer)

	ebitenutil.DebugPrint(screen, "FPS: "+strconv.Itoa(int(ebiten.ActualFPS())))
	ebitenutil.DebugPrintAt(screen, "TRIANGLES RASTERIZED: "+strconv.Itoa(totalTrianglesRasterized), 0, 10)
	ebitenutil.DebugPrintAt(screen, "CORES USED: "+strconv.Itoa(Cores), 0, 20)

	Log.Log(ebiten.ActualFPS())
}

func Launch(renderCallback Callback) {
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

	fmt.Print("\nPlease Select Scene Number: ")
	fmt.Scan(&Scene)

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

	switch Scene {
	case 1:
		var triangle Triangle = Triangle{
			Vertices: [3]Vertex{
				{0, -.5, 0, 1},
				{.5, .5, 0, 1},
				{-.5, .5, 0, 1},
			},

			UV: [3]Vertex{
				{0.5, 1},
				{1, 0},
				{0, 0},
			},

			Color: [3]Vertex{
				{1, 0, 0},
				{0, 1, 0},
				{0, 0, 1},
			},
		}

		var matrix Matrix = TransformationMatrix(Vertex{0, 0, -1}, Vertex{0, 0, 0, 0})
		triangle.Transform(&matrix)
		triangle.Texture = &Brick
		triangle.Shader = &Basic

		Triangles = append(Triangles, triangle)

	case 2:
		for i := 0; i < 1000; i++ {
			var triangle Triangle = Triangle{
				Vertices: [3]Vertex{
					{0, -.5, 0, 1},
					{.5, .5, 0, 1},
					{-.5, .5, 0, 1},
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

			triangle.Texture = &Brick
			triangle.Shader = &Basic

			var matrix Matrix = TransformationMatrix(Vertex{rand.Float32()*16 - 8, rand.Float32()*16 - 8, -12, 0}, Vertex{0, 0, 0, 0})
			triangle.Transform(&matrix)

			Triangles = append(Triangles, triangle)
		}
	case 3:
		Triangles = append(Triangles, Bunny...)

		for i := range Triangles {
			var matrix Matrix = TransformationMatrix(Vertex{0, -1.5, -5, 0}, Vertex{0, 0, 0, 0})
			Triangles[i].Transform(&matrix)
		}
	}

	ebiten.SetWindowSize(Width*Scale, Height*Scale)
	ebiten.SetWindowTitle("Ghetty Engine")
	ebiten.SetTPS(ebiten.SyncWithFPS)
	ebiten.SetScreenClearedEveryFrame(false)
	ebiten.SetVsyncEnabled(false)
	ebiten.SetFullscreen(false)

	Log = NewLogger("raw_data")

	OnRender = renderCallback

	if err := ebiten.RunGame(&Game{}); err != nil {
		log.Fatal(err)
	}

	Log.Close()
}
