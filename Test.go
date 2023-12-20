/*
	This Library Heavily Uses Vek32, Which Is A Convenience Library That Lets You Use SIMD
*/

package main

import (
	"fmt"
	"log"
	"math/rand"
	"strconv"
	"sync"

	"github.com/chewxy/math32"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/viterin/vek/vek32"
)

const (
	X uint8 = 0
	Y uint8 = 1
	Z uint8 = 2
	W uint8 = 3
)

const width, height int = 640, 360
const tileSizeX, tileSizeY = width / 4, height / 3

type Vertex []float32
type Matrix []float32
type Buffer []byte

type TileInfo struct {
	amount int
	tris   []int
}

type Triangle struct {
	v1, v2, v3 Vertex
}

type VertexBundle struct {
	length, capacity int
	buffer           []float32
}

var wg sync.WaitGroup
var mu sync.Mutex

var bundle VertexBundle = VertexBundle{length: 4}
var temporaryBundle VertexBundle = VertexBundle{length: 4}
var finalizedBundle VertexBundle = VertexBundle{length: 4}

var tileData [4][3]VertexBundle

// Used To Compare Vertex Positions With One Another
var compareBundle VertexBundle = VertexBundle{length: 3}

var projectionMatrix, screenSpaceMatrix Matrix
var filterArray []float32

var screenBuffer Buffer

func CreateProjectionMatrix(fov, aspect, near, far float32) Matrix {
	var tangent float32 = math32.Tan((fov * math32.Pi / 180) / 2)

	return Matrix{
		1 / (tangent * aspect), 0, 0, 0,
		0, 1 / tangent, 0, 0,
		0, 0, (far + near) / (near - far), (near * far * 2) / (near - far),
		0, 0, -1, 0,
	}
}

func CreateTransformationMatrix(p Vertex, r Vertex) Matrix {
	return Matrix{
		1 - 2*r[Y]*r[Y] - 2*r[Z]*r[Z], 2*r[X]*r[Y] + 2*r[Z]*r[W], 2*r[X]*r[Z] - 2*r[Y]*r[W], 0,
		2*r[X]*r[Y] - 2*r[Z]*r[W], 1 - 2*r[X]*r[X] - 2*r[Z]*r[Z], 2*r[Y]*r[Z] + 2*r[W]*r[X], 0,
		2*r[X]*r[Z] + 2*r[Y]*r[W], 2*r[Y]*r[Z] - 2*r[W]*r[X], 1 - 2*r[X]*r[X] - 2*r[Y]*r[Y], 0,
		p[X], p[Y], p[Z], 1,
	}
}

func CreateScreenSpaceMatrix(width, height int) Matrix {
	return Matrix{
		float32(width) / 2, 0, 0, 0,
		0, -float32(height) / 2, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
}

var filterMatrix Matrix = Matrix{
	1, 0, 0, 0,
	0, -1, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
}

var xObtainMatrix Matrix = Matrix{
	1, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
}

var yObtainMatrix Matrix = Matrix{
	0, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
}

var identityMatrix Matrix = Matrix{
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1,
}

var tileSpaceMatrix Matrix = Matrix{
	1 / float32(tileSizeX), 0, 0, 0,
	0, 1 / float32(tileSizeY), 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
}

func (m1 *Matrix) MultiplyMatrix(m2 *Matrix) {
	*m1 = vek32.Mat4Mul(*m1, *m2)
}

func (vertexBundle *VertexBundle) AddToBundle(vertex *Vertex) {
	vertexBundle.buffer = append(vertexBundle.buffer, *vertex...)
}

func (vertexBundle *VertexBundle) ObtainVertex(min, max int) Vertex {
	return vertexBundle.buffer[min:max]
}

func (vertexBundle *VertexBundle) ObtainTriangle(index int) Triangle {
	return Triangle{
		vertexBundle.ObtainVertex(index*12, index*12+4), vertexBundle.ObtainVertex(index*12+4, index*12+4+4), vertexBundle.ObtainVertex(index*12+4+4, index*12+4+4+4),
	}
}

func (vertexBundle *VertexBundle) TransformMatrix(matrix *Matrix) {
	vertexBundle.buffer = vek32.MatMul(vertexBundle.buffer, *matrix, vertexBundle.length)
}

func (triangle *Triangle) GetEdgeSpans(x, y float32) (w0, w1, w2 float32) {
	w0 = (triangle.v3[Y]-triangle.v2[Y])*(x-triangle.v2[X]) - (triangle.v3[X]-triangle.v2[X])*(y-triangle.v2[Y])
	w1 = (triangle.v1[Y]-triangle.v3[Y])*(x-triangle.v3[X]) - (triangle.v1[X]-triangle.v3[X])*(y-triangle.v3[Y])
	w2 = (triangle.v2[Y]-triangle.v1[Y])*(x-triangle.v1[X]) - (triangle.v2[X]-triangle.v1[X])*(y-triangle.v1[Y])

	return
}

type Game struct{}

func (vertex *Vertex) ToScreenSpace(width, height int) {
	(*vertex)[X] = (((*vertex)[X] + 1) * float32(width+1)) / 2
	(*vertex)[Y] = ((-(*vertex)[Y] + 1) * float32(height+1)) / 2
}

func (v *Vertex) convertToQuaternion() Vertex {
	var rollRad float32 = (*v)[X] * (math32.Pi / 180)
	var pitchRad float32 = (*v)[Y] * (math32.Pi / 180)
	var yawRad float32 = (*v)[Z] * (math32.Pi / 180)

	rollHalf := rollRad * .5
	pitchHalf := pitchRad * .5
	yawHalf := yawRad * .5

	cosRollHalf := math32.Cos(rollHalf)
	sinRollHalf := math32.Sin(rollHalf)
	cosPitchHalf := math32.Cos(pitchHalf)
	sinPitchHalf := math32.Sin(pitchHalf)
	cosYawHalf := math32.Cos(yawHalf)
	sinYawHalf := math32.Sin(yawHalf)

	// The order of quaternion components is different in this convention (XYZ order).
	x := sinRollHalf*cosPitchHalf*cosYawHalf - cosRollHalf*sinPitchHalf*sinYawHalf
	y := cosRollHalf*sinPitchHalf*cosYawHalf + sinRollHalf*cosPitchHalf*sinYawHalf
	z := cosRollHalf*cosPitchHalf*sinYawHalf - sinRollHalf*sinPitchHalf*cosYawHalf
	w := cosRollHalf*cosPitchHalf*cosYawHalf + sinRollHalf*sinPitchHalf*sinYawHalf

	return Vertex{x, y, z, w}
}

func clamp(value, min, max int) int {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

func clampf(value, min, max float32) float32 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

func RenderTriangle(triangle *Triangle, xTileMin, xTileMax, yTileMin, yTileMax int) {
	//Line-Sweep Rendering
	//We First Order The Y Values Accordingly
	//var triangleCopy Triangle = triangle

	if triangle.v1[Y] > triangle.v2[Y] {
		triangle.v1, triangle.v2 = triangle.v2, triangle.v1
	}

	if triangle.v2[Y] > triangle.v3[Y] {
		triangle.v2, triangle.v3 = triangle.v3, triangle.v2
	}

	if triangle.v1[Y] > triangle.v2[Y] {
		triangle.v1, triangle.v2 = triangle.v2, triangle.v1
	}

	//fmt.Println(triangle.v1[Y], triangle.v2[Y], triangle.v3[Y])

	var splitVertex Vertex = Vertex{
		triangle.v1[X] + ((triangle.v2[Y]-triangle.v1[Y])/(triangle.v3[Y]-triangle.v1[Y]))*(triangle.v3[X]-triangle.v1[X]),
		triangle.v2[Y],
	}

	var invSlope1, invSlope2 float32 = (triangle.v2[X] - triangle.v1[X]) / (triangle.v2[Y] - triangle.v1[Y]),
		(splitVertex[X] - triangle.v1[X]) / (splitVertex[Y] - triangle.v1[Y])

	upperPosition := clamp(int(triangle.v1[Y]), yTileMin, yTileMax)
	lowerPosition := clamp(int(triangle.v2[Y]), yTileMin, yTileMax)

	difference := upperPosition - int(triangle.v1[Y])

	var curX1, curX2 float32 = triangle.v1[X] + (invSlope1 * float32(difference)), triangle.v1[X] + (invSlope2 * float32(difference))

	if invSlope1 > invSlope2 {
		invSlope1, invSlope2 = invSlope2, invSlope1
		curX1, curX2 = curX2, curX1
	}

	for y := upperPosition; y < lowerPosition; y++ {
		for x := clamp(int(curX1), xTileMin, xTileMax); x < clamp(int(curX2), xTileMin, xTileMax); x++ {
			var position int = (y*width + x) * 4
			screenBuffer[position] = 255
		}

		curX1 += invSlope1
		curX2 += invSlope2
	}

	invSlope1 = (triangle.v3[X] - triangle.v2[X]) / (triangle.v3[Y] - triangle.v2[Y])
	invSlope2 = (triangle.v3[X] - splitVertex[X]) / (triangle.v3[Y] - splitVertex[Y])

	upperPosition = clamp(int(triangle.v3[Y]), yTileMin, yTileMax)
	lowerPosition = clamp(int(triangle.v2[Y]), yTileMin, yTileMax) - 1

	difference = int(triangle.v3[Y]) - upperPosition

	curX1 = triangle.v3[X] - (invSlope1 * float32(difference))
	curX2 = triangle.v3[X] - (invSlope2 * float32(difference))

	if invSlope1 < invSlope2 {
		invSlope1, invSlope2 = invSlope2, invSlope1
		curX1, curX2 = curX2, curX1
	}

	for y := upperPosition - 1; y > lowerPosition; y-- {
		for x := clamp(int(curX1), xTileMin, xTileMax); x < clamp(int(curX2), xTileMin, xTileMax); x++ {
			var position int = (y*width + x) * 4
			screenBuffer[position] = 255
		}

		curX1 -= invSlope1
		curX2 -= invSlope2
	}
}

var currentLocation = Vertex{0, 0, -2, 1}
var currentRotation = Vertex{0, 0, 0, 0}

func (g *Game) Update() error {
	if ebiten.IsKeyPressed(ebiten.KeyD) {
		currentLocation[X] += .125 / 2 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyA) {
		currentLocation[X] -= .125 / 2 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyW) {
		currentLocation[Y] += .125 / 2 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyS) {
		currentLocation[Y] -= .125 / 2 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyDown) {
		currentLocation[Z] += .125 / 2 / 2 / 2
	}

	if ebiten.IsKeyPressed(ebiten.KeyUp) {
		currentLocation[Z] -= .125 / 2 / 2 / 2
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	var perCore int = len(bundle.buffer) / 12
	var remainingCore int = perCore % 12

	perCore -= remainingCore

	currentRotation[Z] += 0

	t := CreateTransformationMatrix(currentLocation, Vertex{0, 0, 0, 0})
	r := CreateTransformationMatrix(Vertex{0, 0, 0, 0}, currentRotation.convertToQuaternion())

	s := vek32.Mat4Mul(vek32.Mat4Mul(r, t), projectionMatrix)

	for p := 0; p < 12; p++ {
		wg.Add(1)

		go func(i int) {
			defer wg.Done()
			vek32.MatMul_Into(temporaryBundle.buffer[i*perCore:i*perCore+perCore], bundle.buffer[i*perCore:i*perCore+perCore], s, bundle.length)

			//Clipping Code Goes Here, Will Be Re-Implemented Later

			//Here We Have To Convert To Normalized, Because Of This We Need To Divide By Every Triangle's W Component
			//Since We Don't Know The W Component Beforehand, We Must Do This In A For Loop
			for n := i * perCore; n < i*perCore+perCore; n += 4 {
				vek32.MulNumber_Inplace(temporaryBundle.buffer[n:n+3], 1/temporaryBundle.buffer[n+3])
			}

			//Here We Have To Convert To Screen Space, We Need To Add 1 To Every X & Y Component First Though
			//Since We Know What To Add Beforehand, We Can Just Use This Simple Instruction!
			vek32.Add_Inplace(temporaryBundle.buffer[i*perCore:i*perCore+perCore], filterArray[0:perCore])
			vek32.MatMul_Into(finalizedBundle.buffer[i*perCore:i*perCore+perCore], temporaryBundle.buffer[i*perCore:i*perCore+perCore], screenSpaceMatrix, temporaryBundle.length)

			//Tile Grid Check
			//Get The Position Of The Triangle & Convert It To Tile Space, Round The Value To Avoid Errors
			//Now We Know In What Tile Index Every Vertex Is In!
			vek32.MatMul_Into(temporaryBundle.buffer[i*perCore:i*perCore+perCore], finalizedBundle.buffer[i*perCore:i*perCore+perCore], tileSpaceMatrix, 4)
			//vek32.Round_Inplace(temporaryBundle.buffer[i*perCore : i*perCore+perCore])

			//Now That Everything Is Processed, We Lock The Mutex To Store The Triangle Information To The Tile Array

			mu.Lock()

			for n := i * perCore; n < i*perCore+perCore; n += 12 {
				//We Get The Tile Positions Necessary
				vek32.Gather_Into(compareBundle.buffer, temporaryBundle.buffer, []int{n, n + 4, n + 8})
				var xMin int = clamp(int(math32.Floor(vek32.Min(compareBundle.buffer))), 0, 4)
				var xMax int = clamp(int(math32.Ceil(vek32.Max(compareBundle.buffer))), 0, 4)

				vek32.Gather_Into(compareBundle.buffer, temporaryBundle.buffer, []int{n + 1, n + 5, n + 9})
				var yMin int = clamp(int(math32.Floor(vek32.Min(compareBundle.buffer))), 0, 3)
				var yMax int = clamp(int(math32.Ceil(vek32.Max(compareBundle.buffer))), 0, 3)

				//Finally, We Add The Triangles To Their Respective Tiles
				for y := yMin; y < yMax; y++ {
					for x := xMin; x < xMax; x++ {
						//Because Of Pre-Allocated Memory, We Can't Just Get The Size Through The len() Function
						amount := tileData[x][y].capacity * 12

						//This Used To Append Instead Of Copy, But This Is Much Much Faster Because The Memory Is Pre-Allocated!
						copy(tileData[x][y].buffer[amount:amount+12], finalizedBundle.buffer[n:n+12])
						tileData[x][y].capacity++
					}
				}
			}

			mu.Unlock()
		}(p)
	}

	wg.Wait()

	//Will Be Rewritten!
	for y := 0; y < 3; y++ {
		for x := 0; x < 4; x++ {
			wg.Add(1)

			go func(xPos, yPos int) {
				for t := 0; t < tileData[xPos][yPos].capacity; t++ {
					var triangle Triangle = tileData[xPos][yPos].ObtainTriangle(t)

					RenderTriangle(&triangle, xPos*tileSizeX, xPos*tileSizeX+tileSizeX, yPos*tileSizeY, yPos*tileSizeY+tileSizeY)
				}

				tileData[xPos][yPos].capacity = 0
				wg.Done()
			}(x, y)
		}
	}

	wg.Wait()

	screen.WritePixels(screenBuffer)

	//Clear The Screen Buffer
	for y := 0; y < 3; y++ {
		for x := 0; x < 4; x++ {
			wg.Add(1)

			go func(x, y int) {

				for ty := y * tileSizeY; ty < y*tileSizeY+tileSizeY; ty++ {
					for tx := x * tileSizeX; tx < x*tileSizeX+tileSizeX; tx++ {
						var position int = (ty*width + tx) * 4

						screenBuffer[position] = 0
						screenBuffer[position+1] = 0
						screenBuffer[position+2] = 0
					}
				}

				wg.Done()
			}(x, y)
		}
	}

	wg.Wait()

	ebitenutil.DebugPrint(screen, strconv.Itoa(int(ebiten.ActualTPS())))
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return width, height
}

func main() {
	vek32.SetAcceleration(true)
	fmt.Println(vek32.Info())

	var v1 Vertex = Vertex{
		0, .25, -3, 1,
		-.5, -.75, -3, 1,
		.5, -.5, -3, 1,
	}

	bundle.AddToBundle(&v1)

	for i := 0; i < 1152; i++ {
		t := CreateTransformationMatrix(Vertex{rand.Float32() * 8, rand.Float32() * 4, 0, 0}, Vertex{0, 0, 0, 0})
		//t := CreateTransformationMatrix(Vertex{0, 0, 0, 0}, Vertex{0, 0, 0, 0})

		var v Vertex = vek32.MatMul(v1, t, bundle.length)

		bundle.AddToBundle(&v)
	}

	for y := 0; y < len(tileData[0]); y++ {
		for x := 0; x < len(tileData); x++ {
			tileData[x][y].buffer = vek32.Zeros(len(bundle.buffer))
			tileData[x][y].length = 4
		}
	}

	finalizedBundle.buffer = vek32.Zeros(len(bundle.buffer))
	temporaryBundle.buffer = vek32.Zeros(len(bundle.buffer))
	compareBundle.buffer = vek32.Zeros(3)

	//This Array Will Be Used To Add 1 To Every X & Y Component With SIMD
	filterArray = vek32.MatMul(vek32.Ones(len(bundle.buffer)), filterMatrix, 4)

	projectionMatrix = CreateProjectionMatrix(90, float32(width)/float32(height), .1, 1000)
	screenSpaceMatrix = CreateScreenSpaceMatrix(width, height)

	screenBuffer = make(Buffer, width*height*4)

	game := &Game{}
	ebiten.SetWindowSize(width, height)
	ebiten.SetWindowTitle("Matrix Tests")
	ebiten.SetVsyncEnabled(false)
	ebiten.SetTPS(ebiten.SyncWithFPS)
	if err := ebiten.RunGame(game); err != nil {
		log.Fatal(err)
	}
}
