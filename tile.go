package main

import (
	"sync"
)

const (
	R = 0
	G = 1
	B = 2
)

const (
	BarycentricAlgorithm = 0

	EdgeTestAlgorithm = 1

	SweepLineAlgorithm = 2
)

type Shader func(s, t, w float32) (float32, float32, float32)

type Tile struct {
	Frame []byte

	Triangles []*ProcessedTriangle

	X, Y int
}

var AlgorithmUsed int = SweepLineAlgorithm

var Pitch int
var BytesPerPixel int
var WaitGroup sync.WaitGroup
var Mutex sync.Mutex

func (tile *Tile) Barycentric(triangle *ProcessedTriangle) {
	var xMin, yMin, xMax, yMax int = int(Clamp(triangle.Bounds[XMIN], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMIN], tile.Y, tile.Y+TileYSize)),
		int(Clamp(triangle.Bounds[XMAX], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMAX], tile.Y, tile.Y+TileYSize))

	for y := yMin; y < yMax; y++ {
		for x := xMin; x < xMax; x++ {
			if inside, s, t, w := triangle.Inside(x, y); inside {
				var r, g, b float32 = triangle.Triangle.Shader(
					w*triangle.Triangle.Color[0][R]+s*triangle.Triangle.Color[1][R]+t*triangle.Triangle.Color[2][R],
					w*triangle.Triangle.Color[0][G]+s*triangle.Triangle.Color[1][G]+t*triangle.Triangle.Color[2][G],
					w*triangle.Triangle.Color[0][B]+s*triangle.Triangle.Color[1][B]+t*triangle.Triangle.Color[2][B],
				)

				tile.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
			}
		}
	}
}

func (tile *Tile) EdgeTest(triangle *ProcessedTriangle) {
	var xMin, yMin, xMax, yMax int = int(Clamp(triangle.Bounds[XMIN], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMIN], tile.Y, tile.Y+TileYSize)),
		int(Clamp(triangle.Bounds[XMAX], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMAX], tile.Y, tile.Y+TileYSize))

	for y := yMin; y < yMax; y++ {
		for x := xMin; x < xMax; x++ {

			var w0, w1, w2 float32 = triangle.Triangle.EdgeSpan(x, y)

			if w0 >= 0 && w1 >= 0 && w2 >= 0 {
				var s, t, w float32 = triangle.Barycentric(x, y)
				var r, g, b float32 = triangle.Triangle.Shader(s, t, w)

				tile.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
			}
		}
	}
}

func (tile *Tile) SweepLine(triangle *ProcessedTriangle) {
	var invSlope1 float32 = (triangle.Triangle.Vertices[1][X] - triangle.Triangle.Vertices[0][X]) /
		(triangle.Triangle.Vertices[1][Y] - triangle.Triangle.Vertices[0][Y])

	var invSlope2 float32 = (triangle.Split - triangle.Triangle.Vertices[0][X]) /
		(triangle.Triangle.Vertices[1][Y] - triangle.Triangle.Vertices[0][Y])

	var clampedUp float32 = Clamp(triangle.Triangle.Vertices[0][Y], tile.Y, tile.Y+TileYSize)
	var clampedMiddle float32 = Clamp(triangle.Triangle.Vertices[1][Y], tile.Y, tile.Y+TileYSize)
	var clampedDown float32 = Clamp(triangle.Triangle.Vertices[2][Y], tile.Y, tile.Y+TileYSize)

	var difference float32 = clampedUp - triangle.Triangle.Vertices[0][Y]
	var curX1, curX2 float32 = triangle.Triangle.Vertices[0][X] + invSlope1*difference,
		triangle.Triangle.Vertices[0][X] + invSlope2*difference

	for y := int(clampedUp); y < int(clampedMiddle); y++ {
		var clampedLeft, clampedRight int = int(Clamp(curX1, tile.X, tile.X+TileXSize)),
			int(Clamp(curX2, tile.X, tile.X+TileXSize))

		for x := clampedLeft; x < clampedRight; x++ {
			var s, t, w float32 = triangle.Barycentric(x, y)
			var r, g, b float32 = triangle.Triangle.Shader(s, t, w)

			tile.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
		}

		curX1 += invSlope1
		curX2 += invSlope2
	}

	invSlope1 = (triangle.Triangle.Vertices[2][X] - triangle.Triangle.Vertices[1][X]) /
		(triangle.Triangle.Vertices[2][Y] - triangle.Triangle.Vertices[1][Y])

	invSlope2 = (triangle.Triangle.Vertices[2][X] - triangle.Split) /
		(triangle.Triangle.Vertices[2][Y] - triangle.Triangle.Vertices[1][Y])

	difference = triangle.Triangle.Vertices[2][Y] - clampedDown
	curX1, curX2 = triangle.Triangle.Vertices[2][X]-invSlope1*difference, triangle.Triangle.Vertices[2][X]-invSlope2*difference

	for y := int(clampedDown) - 1; y > int(clampedMiddle)-1; y-- {
		var clampedLeft, clampedRight int = int(Clamp(curX1, tile.X, tile.X+TileXSize)),
			int(Clamp(curX2, tile.X, tile.X+TileXSize))

		for x := clampedLeft; x < clampedRight; x++ {
			var s, t, w float32 = triangle.Barycentric(x, y)
			var r, g, b float32 = triangle.Triangle.Shader(s, t, w)

			tile.Set(x, y, byte(r*255), byte(g*255), byte(b*255))
		}

		curX1 -= invSlope1
		curX2 -= invSlope2
	}
}

func (tile *Tile) Rasterize() {
	switch AlgorithmUsed {
	case BarycentricAlgorithm:
		for index := range tile.Triangles {
			tile.Barycentric(tile.Triangles[index])
		}
	case SweepLineAlgorithm:
		for index := range tile.Triangles {
			tile.SweepLine(tile.Triangles[index])
		}
	case EdgeTestAlgorithm:
		for index := range tile.Triangles {
			tile.EdgeTest(tile.Triangles[index])
		}
	}

	tile.Triangles = nil
}

func (tile *Tile) Add(triangle *ProcessedTriangle) {
	tile.Triangles = append(tile.Triangles, triangle)
}

func (tile *Tile) Set(x, y int, r, g, b byte) {
	var position int = (y*Width + x) * 4

	tile.Frame[position+R], tile.Frame[position+G], tile.Frame[position+B] = r, g, b
}

func (tile *Tile) Clear(r, g, b byte) {
	for y := tile.Y; y < tile.Y+TileYSize; y++ {
		for x := tile.X; x < tile.X+TileXSize; x++ {
			tile.Set(x, y, r, g, b)
		}
	}
}
