package main

import (
	"fmt"
	_ "image/jpeg"
	_ "image/png"
	"log"
	"math"
	"runtime"
	"strconv"
	"sync"

	"github.com/chewxy/math32"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

type Vertex2D struct {
	x, y float32
}

type Vertex3D struct {
	x, y, z float32
}

type Vertex4D struct {
	x, y, z, w float32
}

type Triangle struct {
	vertices [3]Vertex3D
	uv       [3]Vertex2D
}

type ComputedTriangle struct {
	vertices [3]Vertex4D
	uv       [3]Vertex2D
}

type Buffer []byte
type FloatBuffer []float32
type Matrix [][]float32

var screenBuffer Buffer
var depthBuffer FloatBuffer
var cores int

//var depthBuffer Buffer

var basicTriangle, basicTriangle2 Triangle
var width, height int = 320, 180
var aspectRatio float32 = float32(width) / float32(height)
var wg sync.WaitGroup

var cobble *ebiten.Image
var cobble_buffer Buffer
var fov float32 = 120

var projectionMatrix Matrix

func (vertex *Vertex4D) convertToScreenSpace() {
	vertex.x = (((vertex.x + 1) * float32(width+1)) / 2)
	vertex.y = (((-vertex.y + 1) * float32(height+1)) / 2)
}

func (v1 *Vertex4D) crossProduct(v2 *Vertex4D) float32 {
	return (v1.x * v2.y) - (v1.y * v2.x)
}

func (vertex *Vertex3D) convertToMatrix() Matrix {
	return Matrix{{vertex.x, vertex.y, vertex.z, 1}}
}

func (m1 *Matrix) multiplyMatrix(m2 *Matrix) (result Matrix) {
	result = make(Matrix, len(*m1))

	for i := range result {
		result[i] = make([]float32, len((*m2)[0]))

		for j := range result[i] {
			for k := 0; k < len((*m1)[0]); k++ {
				result[i][j] += (*m1)[i][k] * (*m2)[k][j]
			}
		}
	}

	return
}

func (matrix *Matrix) convertToVertex() Vertex4D {
	return Vertex4D{(*matrix)[0][0], (*matrix)[0][1], (*matrix)[0][2], (*matrix)[0][3]}
}

func (triangle *Triangle) multiplyMatrix(m2 *Matrix) (result ComputedTriangle) {
	var tm1, tm2, tm3 Matrix = triangle.vertices[0].convertToMatrix(), triangle.vertices[1].convertToMatrix(), triangle.vertices[2].convertToMatrix()
	tm1, tm2, tm3 = tm1.multiplyMatrix(m2), tm2.multiplyMatrix(m2), tm3.multiplyMatrix(m2)

	result.vertices[0] = tm1.convertToVertex()
	result.vertices[1] = tm2.convertToVertex()
	result.vertices[2] = tm3.convertToVertex()

	result.uv = triangle.uv

	return
}

func createProjectionMatrix(fov, aspect, near, far float32) Matrix {
	var tangent float32 = math32.Tan((fov * (math.Pi / 180)) / 2)

	return Matrix{
		{1 / (tangent * aspect), 0, 0, 0},
		{0, 1 / tangent, 0, 0},
		{0, 0, (far + near) / (near - far), (near * far * 2) / (near - far)},
		{0, 0, -1, 0},
	}
}

func (triangle *ComputedTriangle) spanningVectors() (vs1, vs2 Vertex4D) {
	vs1 = Vertex4D{triangle.vertices[1].x - triangle.vertices[0].x, triangle.vertices[1].y - triangle.vertices[0].y, 0, 1}
	vs2 = Vertex4D{triangle.vertices[2].x - triangle.vertices[0].x, triangle.vertices[2].y - triangle.vertices[0].y, 0, 1}

	return
}

func (triangle *ComputedTriangle) bounds() (minX, minY, maxX, maxY int) {
	maxX = int(math32.Max(triangle.vertices[0].x, math32.Max(triangle.vertices[1].x, triangle.vertices[2].x)))
	maxY = int(math32.Max(triangle.vertices[0].y, math32.Max(triangle.vertices[1].y, triangle.vertices[2].y)))

	minX = int(math32.Min(triangle.vertices[0].x, math32.Min(triangle.vertices[1].x, triangle.vertices[2].x)))
	minY = int(math32.Min(triangle.vertices[0].y, math32.Min(triangle.vertices[1].y, triangle.vertices[2].y)))

	return
}

func (triangle *ComputedTriangle) barycentricCoordinates(vs1, vs2 *Vertex4D, x, y *int, span *float32) (s, t, w float32) {
	var q Vertex4D = Vertex4D{float32(*x) - triangle.vertices[0].x, float32(*y) - triangle.vertices[0].y, 0, 1}

	s = q.crossProduct(vs2) * (1.0 / *span)
	t = vs1.crossProduct(&q) * (1.0 / *span)

	w = 1 - s - t

	return
}

func (triangle *ComputedTriangle) convertToScreenSpace() {
	triangle.vertices[0].convertToScreenSpace()
	triangle.vertices[1].convertToScreenSpace()
	triangle.vertices[2].convertToScreenSpace()
}

func (v *Vertex4D) convertToNormalized() {
	v.x /= v.w
	v.y /= v.w
	v.z /= v.w
}

func (v1 *Vertex4D) interpolate(v2 *Vertex4D, factor float32) Vertex4D {
	return Vertex4D{
		v1.x*(1-factor) + v2.x*factor,
		v1.y*(1-factor) + v2.y*factor,
		v1.z*(1-factor) + v2.z*factor,
		v1.w*(1-factor) + v2.w*factor,
	}
}

func (v1 *Vertex2D) interpolate(v2 *Vertex2D, factor float32) Vertex2D {
	return Vertex2D{
		v1.x*(1-factor) + v2.x*factor,
		v1.y*(1-factor) + v2.y*factor,
	}
}

func (triangle *ComputedTriangle) convertToNormalizedCoordinates() {
	triangle.vertices[0].convertToNormalized()
	triangle.vertices[1].convertToNormalized()
	triangle.vertices[2].convertToNormalized()
}

func (buffer *Buffer) clearScreen() {
	for x := 0; x < len(*buffer); x++ {
		(*buffer)[x] = 0
	}
}

func (buffer *FloatBuffer) clearDepth() {
	for x := 0; x < len(*buffer); x++ {
		(*buffer)[x] = math32.MaxFloat32
	}
}

func (triangle *ComputedTriangle) renderToScreen(buffer *Buffer, depthBuffer *FloatBuffer, texture *Buffer, image *ebiten.Image) {
	var vs1, vs2 Vertex4D = triangle.spanningVectors()
	var span float32 = vs1.crossProduct(&vs2)

	var minX, minY, maxX, maxY int = triangle.bounds()

	var at Vertex3D = Vertex3D{triangle.uv[0].x / triangle.vertices[0].w, triangle.uv[0].y / triangle.vertices[0].w, 1 / triangle.vertices[0].w}
	var bt Vertex3D = Vertex3D{triangle.uv[1].x / triangle.vertices[1].w, triangle.uv[1].y / triangle.vertices[1].w, 1 / triangle.vertices[1].w}
	var ct Vertex3D = Vertex3D{triangle.uv[2].x / triangle.vertices[2].w, triangle.uv[2].y / triangle.vertices[2].w, 1 / triangle.vertices[2].w}

	//var xAmount int = maxX - minX
	//var yAmount int = maxY - minY

	var chunkSize int = (width * height) / cores

	//fmt.Println(maxX - minX)

	for i := 0; i < cores; i++ {
		wg.Add(1)

		go func(section int) {
			for p := section * chunkSize; p < section*chunkSize+chunkSize; p++ {
				var x int = 1 + p%(width)
				var y int = 1 + (p/(width))%(height)

				if x >= minX && x <= maxX && y >= minY && y <= maxY {
					var s, t, w float32 = triangle.barycentricCoordinates(&vs1, &vs2, &x, &y, &span)

					if s >= 0 && t >= 0 && s+t <= 1 {
						var depth float32 = s*triangle.vertices[0].z + t*triangle.vertices[1].z + w*triangle.vertices[2].z

						if depth <= (*depthBuffer)[((y-1)*width+(x-1))] {
							var location int = ((y-1)*width + (x - 1)) * 4

							var wt float32 = w*at.z + s*bt.z + t*ct.z

							var uvX float32 = (w*at.x + s*bt.x + t*ct.x) / wt
							var uvY float32 = (w*at.y + s*bt.y + t*ct.y) / wt

							var tx int = int(uvX * float32(image.Bounds().Dx()))
							var ty int = int(uvY * float32(image.Bounds().Dy()))

							var colorLocation int = ((ty*image.Bounds().Dx() + tx) * 4) % (image.Bounds().Dx() * image.Bounds().Dy() * 4)

							(*depthBuffer)[((y-1)*width + (x - 1))] = depth

							(*buffer)[location] = (*texture)[colorLocation]
							(*buffer)[location+1] = (*texture)[colorLocation+1]
							(*buffer)[location+2] = (*texture)[colorLocation+2]
						}
					}
				}

			}

			wg.Done()
		}(i)

	}

}

func clip_axis(vertices *[]Vertex4D, uv *[]Vertex2D, factor float32, axis int) (data []Vertex4D, uvData []Vertex2D) {
	var previousVertex *Vertex4D = &(*vertices)[len(*vertices)-1]
	var previousUV *Vertex2D = &(*uv)[len(*uv)-1]

	var previousComponent float32 = factor
	var previousInside bool

	switch axis {
	case 0:
		previousComponent *= previousVertex.x
	case 1:
		previousComponent *= previousVertex.y
	case 2:
		previousComponent *= previousVertex.z
	}

	if previousComponent <= previousVertex.w {
		previousInside = true
	}

	for v := 0; v < len(*vertices); v++ {
		var currentVertex *Vertex4D = &(*vertices)[v]
		var currentUV *Vertex2D = &(*uv)[v]

		var currentComponent float32 = factor
		var currentInside bool

		switch axis {
		case 0:
			currentComponent *= currentVertex.x
		case 1:
			currentComponent *= currentVertex.y
		case 2:
			currentComponent *= currentVertex.z
		}
		if currentComponent <= currentVertex.w {
			currentInside = true
		}

		if currentInside != previousInside {
			var factor float32 = (previousVertex.w - previousComponent) / ((previousVertex.w - previousComponent) - (currentVertex.w - currentComponent))

			data = append(data, previousVertex.interpolate(currentVertex, factor))
			uvData = append(uvData, previousUV.interpolate(currentUV, factor))
		}

		if currentInside {
			data = append(data, *currentVertex)
			uvData = append(uvData, *currentUV)
		}

		previousVertex = currentVertex
		previousComponent = currentComponent
		previousInside = currentInside
		previousUV = currentUV
	}

	return
}

func (t *ComputedTriangle) clip() (triangles []ComputedTriangle) {
	var vertices []Vertex4D = t.vertices[:]
	var uvdat []Vertex2D = t.uv[:]

	for i := 0; i < 2; i++ {
		if len(vertices) > 0 {
			vertices, uvdat = clip_axis(&vertices, &uvdat, 1, i)

			if len(vertices) > 0 {
				vertices, uvdat = clip_axis(&vertices, &uvdat, -1, i)
			} else {
				break
			}
		} else {
			break
		}
	}

	if len(vertices) > 0 {
		for index := 0; index < len(vertices)-2; index++ {
			triangles = append(triangles, ComputedTriangle{
				[3]Vertex4D{vertices[0], vertices[index+1], vertices[index+2]},
				[3]Vertex2D{uvdat[0], uvdat[index+1], uvdat[index+2]}})
		}
	}

	return
}

type Game struct{}

func (g *Game) Update() error {
	if ebiten.IsKeyPressed(ebiten.KeyRight) {
		basicTriangle.vertices[0].x += .001
		basicTriangle.vertices[1].x += .001
		basicTriangle.vertices[2].x += .001
	}

	if ebiten.IsKeyPressed(ebiten.KeyLeft) {
		basicTriangle.vertices[0].x -= .001
		basicTriangle.vertices[1].x -= .001
		basicTriangle.vertices[2].x -= .001
	}

	if ebiten.IsKeyPressed(ebiten.KeyUp) {
		basicTriangle.vertices[0].y += .001
		basicTriangle.vertices[1].y += .001
		basicTriangle.vertices[2].y += .001
	}

	if ebiten.IsKeyPressed(ebiten.KeyDown) {
		basicTriangle.vertices[0].y -= .001
		basicTriangle.vertices[1].y -= .001
		basicTriangle.vertices[2].y -= .001
	}

	if ebiten.IsKeyPressed(ebiten.KeyW) {
		basicTriangle.vertices[0].z -= .01
		basicTriangle.vertices[1].z -= .01
		basicTriangle.vertices[2].z -= .01
	}

	if ebiten.IsKeyPressed(ebiten.KeyS) {
		basicTriangle.vertices[0].z += .01
		basicTriangle.vertices[1].z += .01
		basicTriangle.vertices[2].z += .01
	}

	if ebiten.IsKeyPressed(ebiten.KeyD) {
		basicTriangle.vertices[1].z += .01
		basicTriangle.vertices[2].z += .01
	}

	if ebiten.IsKeyPressed(ebiten.KeyA) {
		basicTriangle.vertices[1].z -= .01
		basicTriangle.vertices[2].z -= .01
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	if cobble_buffer == nil {
		cobble_buffer = make(Buffer, cobble.Bounds().Dx()*cobble.Bounds().Dy()*4)
		cobble.ReadPixels(cobble_buffer)
	}

	var convertedTriangle ComputedTriangle = basicTriangle.multiplyMatrix(&projectionMatrix)
	var clippedTriangles []ComputedTriangle = convertedTriangle.clip()

	for i := 0; i < len(clippedTriangles); i++ {

		clippedTriangles[i].convertToNormalizedCoordinates()
		clippedTriangles[i].convertToScreenSpace()

		clippedTriangles[i].renderToScreen(&screenBuffer, &depthBuffer, &cobble_buffer, cobble)
		wg.Wait()
	}

	var convertedTriangle2 ComputedTriangle = basicTriangle2.multiplyMatrix(&projectionMatrix)
	var clippedTriangles2 []ComputedTriangle = convertedTriangle2.clip()

	for i := 0; i < len(clippedTriangles2); i++ {

		clippedTriangles2[i].convertToNormalizedCoordinates()
		clippedTriangles2[i].convertToScreenSpace()

		clippedTriangles2[i].renderToScreen(&screenBuffer, &depthBuffer, &cobble_buffer, cobble)
		wg.Wait()
	}

	screen.WritePixels(screenBuffer)
	screenBuffer.clearScreen()
	depthBuffer.clearDepth()

	ebitenutil.DebugPrint(screen, strconv.Itoa(int(ebiten.ActualFPS())))
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return width, height
}

func init() {
	var err error
	cobble, _, err = ebitenutil.NewImageFromFile("cobble.png")

	if err != nil {
		log.Fatal(err)
	}
}

func main() {
	fmt.Println("Initializing Polygon Core")

	ebiten.SetWindowSize(1280, 720)
	ebiten.SetWindowTitle("Polygon Core - V2")
	ebiten.SetVsyncEnabled(false)
	ebiten.SetTPS(ebiten.SyncWithFPS)
	ebiten.SetScreenClearedEveryFrame(false)
	ebiten.SetWindowResizingMode(ebiten.WindowResizingModeEnabled)

	screenBuffer = make(Buffer, width*height*4)
	depthBuffer = make(FloatBuffer, width*height)
	cores = runtime.NumCPU() - 1
	fmt.Println("Screen Buffer Initialized")

	projectionMatrix = createProjectionMatrix(fov, aspectRatio, .1, 1000)

	basicTriangle.vertices[0] = Vertex3D{0, .5, -2}
	basicTriangle.vertices[1] = Vertex3D{-.5, -.5, -2}
	basicTriangle.vertices[2] = Vertex3D{.5, -.5, -2}

	basicTriangle2.vertices[0] = Vertex3D{0, .125, -2}
	basicTriangle2.vertices[1] = Vertex3D{-.125, -.125, -2}
	basicTriangle2.vertices[2] = Vertex3D{.125, -.125, -2}

	basicTriangle.uv[0] = Vertex2D{0, 0}
	basicTriangle.uv[1] = Vertex2D{2, 0}
	basicTriangle.uv[2] = Vertex2D{2, 2}

	basicTriangle2.uv[0] = Vertex2D{0, 0}
	basicTriangle2.uv[1] = Vertex2D{2, 0}
	basicTriangle2.uv[2] = Vertex2D{2, 2}

	if err := ebiten.RunGameWithOptions(&Game{}, &ebiten.RunGameOptions{GraphicsLibrary: ebiten.GraphicsLibraryOpenGL, InitUnfocused: false, ScreenTransparent: false, SkipTaskbar: false}); err != nil {
		log.Fatal(err)
	}
}
