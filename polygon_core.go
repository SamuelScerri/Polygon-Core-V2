package main

import (
	"fmt"
	_ "image/png"
	"log"
	"strconv"

	"github.com/chewxy/math32"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
)

type Vertex4D struct {
	x, y, z, w float32
}

type Triangle struct {
	vertices [3]Vertex4D
}

type Buffer []byte
type Matrix [][]float32

var screenBuffer Buffer

//var depthBuffer Buffer

var basicTriangle Triangle
var width, height int = 640, 360
var aspectRatio float32 = float32(width) / float32(height)

var cobble *ebiten.Image
var cobble_buffer Buffer

var projectionMatrix Matrix

func (vertex *Vertex4D) convertToScreenSpace() {
	vertex.x = ((vertex.x + 1) * float32(width)) / 2
	vertex.y = ((-vertex.y + 1) * float32(height)) / 2
}

func (v1 *Vertex4D) crossProduct(v2 *Vertex4D) float32 {
	return (v1.x * v2.y) - (v1.y * v2.x)
}

func (vertex *Vertex4D) convertToMatrix() Matrix {
	return Matrix{{vertex.x, vertex.y, vertex.z, vertex.w}}
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

func (triangle *Triangle) multiplyMatrix(m2 *Matrix) (result Triangle) {
	var tm1, tm2, tm3 Matrix = triangle.vertices[0].convertToMatrix(), triangle.vertices[1].convertToMatrix(), triangle.vertices[2].convertToMatrix()
	tm1, tm2, tm3 = tm1.multiplyMatrix(m2), tm2.multiplyMatrix(m2), tm3.multiplyMatrix(m2)

	result.vertices[0] = tm1.convertToVertex()
	result.vertices[1] = tm2.convertToVertex()
	result.vertices[2] = tm3.convertToVertex()

	return
}

func createProjectionMatrix(fov, aspect, near, far float32) Matrix {
	var tangent float32 = math32.Tan(fov / 2)

	return Matrix{
		{1 / (tangent * aspect), 0, 0, 0},
		{0, 1 / tangent, 0, 0},
		{0, 0, (far + near) / (near - far), (near * far * 2) / (near - far)},
		{0, 0, -1, 0},
	}
}

func (triangle *Triangle) spanningVectors() (vs1, vs2 Vertex4D) {
	vs1 = Vertex4D{triangle.vertices[1].x - triangle.vertices[0].x, triangle.vertices[1].y - triangle.vertices[0].y, 0, 1}
	vs2 = Vertex4D{triangle.vertices[2].x - triangle.vertices[0].x, triangle.vertices[2].y - triangle.vertices[0].y, 0, 1}

	return
}

func (triangle *Triangle) bounds() (minX, minY, maxX, maxY int) {
	maxX = int(math32.Max(triangle.vertices[0].x, math32.Max(triangle.vertices[1].x, triangle.vertices[2].x)))
	maxY = int(math32.Max(triangle.vertices[0].y, math32.Max(triangle.vertices[1].y, triangle.vertices[2].y)))

	minX = int(math32.Min(triangle.vertices[0].x, math32.Min(triangle.vertices[1].x, triangle.vertices[2].x)))
	minY = int(math32.Min(triangle.vertices[0].y, math32.Min(triangle.vertices[1].y, triangle.vertices[2].y)))

	return
}

func (triangle *Triangle) barycentricCoordinates(vs1, vs2 *Vertex4D, x, y, span *float32) (s, t, w float32) {
	var q Vertex4D = Vertex4D{*x - triangle.vertices[0].x, *y - triangle.vertices[0].y, 0, 1}

	s = q.crossProduct(vs2) / *span
	t = vs1.crossProduct(&q) / *span

	w = 1 - s - t

	return
}

func (triangle *Triangle) convertToScreenSpace() {
	triangle.vertices[0].convertToScreenSpace()
	triangle.vertices[1].convertToScreenSpace()
	triangle.vertices[2].convertToScreenSpace()
}

func (v *Vertex4D) convertToNormalized() {
	v.x /= v.w
	v.y /= v.w
	v.z /= v.w
}

func (triangle *Triangle) convertToNormalizedCoordinates() {
	triangle.vertices[0].convertToNormalized()
	triangle.vertices[1].convertToNormalized()
	triangle.vertices[2].convertToNormalized()
}

func (buffer *Buffer) clearScreen() {
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			var location int = (y*width + x) * 4

			(*buffer)[location] = 64
			(*buffer)[location+1] = 64
			(*buffer)[location+2] = 64
		}
	}
}

func (triangle *Triangle) renderToScreen(buffer *Buffer, texture *Buffer, image *ebiten.Image) {
	var vs1, vs2 Vertex4D = triangle.spanningVectors()
	var span float32 = vs1.crossProduct(&vs2)

	var minX, minY, maxX, maxY = triangle.bounds()

	for x := minX; x < maxX; x++ {
		for y := minY; y < maxY; y++ {
			var convertedX, convertedY float32 = float32(x), float32(y)

			var s, t, w float32 = triangle.barycentricCoordinates(&vs1, &vs2, &convertedX, &convertedY, &span)

			if s >= 0 && t >= 0 && s+t <= 1 {
				var location int = (y*width + x) * 4

				var at Vertex4D = Vertex4D{0, 0, 1, triangle.vertices[0].w}
				var bt Vertex4D = Vertex4D{2, 0, 1, triangle.vertices[1].w}
				var ct Vertex4D = Vertex4D{2, 2, 1, triangle.vertices[2].w}

				at.convertToNormalized()
				bt.convertToNormalized()
				ct.convertToNormalized()

				var wt float32 = w*at.z + s*bt.z + t*ct.z

				var uvX float32 = (w*at.x + s*bt.x + t*ct.x) / wt
				var uvY float32 = (w*at.y + s*bt.y + t*ct.y) / wt

				var tx int = int(uvX * float32(image.Bounds().Dx()))
				var ty int = int(uvY * float32(image.Bounds().Dy()))

				var colorLocation int = ((ty*image.Bounds().Dx() + tx) * 4) % (image.Bounds().Dx() * image.Bounds().Dy() * 4)

				(*buffer)[location] = (*texture)[colorLocation]
				(*buffer)[location+1] = (*texture)[colorLocation+1]
				(*buffer)[location+2] = (*texture)[colorLocation+2]
			}
		}
	}
}

type Game struct{}

func (g *Game) Update() error {
	basicTriangle.vertices[0].z -= .001
	basicTriangle.vertices[1].z -= .001
	basicTriangle.vertices[2].z -= .001

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	if cobble_buffer == nil {
		cobble_buffer = make(Buffer, cobble.Bounds().Dx()*cobble.Bounds().Dy()*4)
		cobble.ReadPixels(cobble_buffer)
	}

	var convertedTriangle Triangle = basicTriangle.multiplyMatrix(&projectionMatrix)

	convertedTriangle.convertToNormalizedCoordinates()
	convertedTriangle.convertToScreenSpace()

	convertedTriangle.renderToScreen(&screenBuffer, &cobble_buffer, cobble)
	screen.WritePixels(screenBuffer)
	screenBuffer.clearScreen()

	ebitenutil.DebugPrint(screen, strconv.Itoa(int(ebiten.ActualFPS())))

}

func (g *Game) Layout(outsideWidth, outsideHeight int) (screenWidth, screenHeight int) {
	return width, height
}

func init() {
	var err error
	cobble, _, err = ebitenutil.NewImageFromFile("checkerboard.png")

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
	fmt.Println("Screen Buffer Initialized")

	//depthBuffer = make(Buffer, width*height)
	//fmt.Println("Depth Buffer Initialized")

	projectionMatrix = createProjectionMatrix(90, aspectRatio, .1, 1000)

	basicTriangle.vertices[0] = Vertex4D{0, .5, -8, 1}
	basicTriangle.vertices[1] = Vertex4D{-.5, -.5, -8, 1}
	basicTriangle.vertices[2] = Vertex4D{.5, -.5, -2, 1}

	if err := ebiten.RunGameWithOptions(&Game{}, &ebiten.RunGameOptions{GraphicsLibrary: ebiten.GraphicsLibraryOpenGL, InitUnfocused: false, ScreenTransparent: false, SkipTaskbar: false}); err != nil {
		log.Fatal(err)
	}
}
