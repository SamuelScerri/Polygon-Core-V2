package main

import (
	"math"
	"sync"
)

const (
	BarycentricAlgorithm = 0

	EdgeTestAlgorithm = 1

	SweepLineAlgorithm = 2
)

type Tile struct {
	Frame []byte
	Depth []float32

	Triangles []*ProcessedTriangle

	X, Y int
}

var AlgorithmUsed int = SweepLineAlgorithm

var Pitch int
var BytesPerPixel int
var WaitGroup sync.WaitGroup
var Mutex sync.Mutex

func (tile *Tile) Barycentric(triangle *ProcessedTriangle) {
	var xMin, yMin, xMax, yMax int = int(Clamp(triangle.Bounds[XMin], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMin], tile.Y, tile.Y+TileYSize)),
		int(Clamp(triangle.Bounds[XMax], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMax], tile.Y, tile.Y+TileYSize))

	for y := yMin; y < yMax; y++ {
		for x := xMin; x < xMax; x++ {
			if inside, w, s, t := triangle.Inside(x, y); inside {
				var depth float32 = w*triangle.Triangle.Vertices[0][Z] + s*triangle.Triangle.Vertices[1][Z] + t*triangle.Triangle.Vertices[2][Z]

				if position := tile.ConvertPosition(x, y); depth < tile.Depth[position] {

					if triangle.Triangle.Texture == nil {
						//var r, g, b float32 = triangle.Triangle.Shader(triangle.Triangle.Interpolate(w, s, t))
						//tile.Set(position, byte(r*255), byte(g*255), byte(b*255), depth)
					} else {
						var wt float32 = 1 / (w*triangle.Triangle.UV[0][Z] + s*triangle.Triangle.UV[1][Z] + t*triangle.Triangle.UV[2][Z])

						var uvx float32 = (w*triangle.Triangle.UV[0][X] + s*triangle.Triangle.UV[1][X] + t*triangle.Triangle.UV[2][X]) * wt
						var uvy float32 = (w*triangle.Triangle.UV[0][Y] + s*triangle.Triangle.UV[1][Y] + t*triangle.Triangle.UV[2][Y]) * wt

						var tx int = int(uvx * float32(triangle.Triangle.Texture.Width))
						var ty int = int((1 - uvy) * float32(triangle.Triangle.Texture.Height))

						r, g, b, _ := triangle.Triangle.Texture.Get(triangle.Triangle.Texture.ConvertPosition(tx, ty))

						tile.Set(position, r, g, b, depth)
					}

				}
			}
		}
	}
}

func (tile *Tile) EdgeTest(triangle *ProcessedTriangle) {
	var xMin, yMin, xMax, yMax int = int(Clamp(triangle.Bounds[XMin], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMin], tile.Y, tile.Y+TileYSize)),
		int(Clamp(triangle.Bounds[XMax], tile.X, tile.X+TileXSize)),
		int(Clamp(triangle.Bounds[YMax], tile.Y, tile.Y+TileYSize))

	for y := yMin; y < yMax; y++ {
		for x := xMin; x < xMax; x++ {

			var w0, w1, w2 float32 = triangle.Triangle.EdgeSpan(x, y)

			if w0 >= 0 && w1 >= 0 && w2 >= 0 {
				var w, s, t float32 = triangle.Barycentric(x, y)
				var depth float32 = w*triangle.Triangle.Vertices[0][Z] + s*triangle.Triangle.Vertices[1][Z] + t*triangle.Triangle.Vertices[2][Z]

				if position := tile.ConvertPosition(x, y); depth < tile.Depth[position] {
					if triangle.Triangle.Texture == nil {
						//var r, g, b float32 = triangle.Triangle.Shader(triangle.Triangle.Interpolate(w, s, t))
						//tile.Set(position, byte(r*255), byte(g*255), byte(b*255), depth)
					} else {
						var wt float32 = 1 / (w*triangle.Triangle.UV[0][Z] + s*triangle.Triangle.UV[1][Z] + t*triangle.Triangle.UV[2][Z])

						var uvx float32 = (w*triangle.Triangle.UV[0][X] + s*triangle.Triangle.UV[1][X] + t*triangle.Triangle.UV[2][X]) * wt
						var uvy float32 = (w*triangle.Triangle.UV[0][Y] + s*triangle.Triangle.UV[1][Y] + t*triangle.Triangle.UV[2][Y]) * wt

						var tx int = int(uvx * float32(triangle.Triangle.Texture.Width))
						var ty int = int((1 - uvy) * float32(triangle.Triangle.Texture.Height))

						r, g, b, _ := triangle.Triangle.Texture.Get(triangle.Triangle.Texture.ConvertPosition(tx, ty))

						tile.Set(position, r, g, b, depth)
					}
				}
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
			//var w, s, t float32 = triangle.Barycentric(x, y)
			//var r, g, b float32 = triangle.Triangle.Shader(s, t, w)

			//tile.Set(tile.ConvertPosition(x, y),
			//	byte(r*255), byte(g*255), byte(b*255), 0)
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
			//var s, t, w float32 = triangle.Barycentric(x, y)
			//var r, g, b float32 = triangle.Triangle.Shader(s, t, w)

			//tile.Set(tile.ConvertPosition(x, y),
			//	byte(r*255), byte(g*255), byte(b*255), 0)
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

func (tile *Tile) ConvertPosition(x, y int) int {
	return y*Width + x
}

func (tile *Tile) Set(position int, r, g, b byte, depth float32) {
	var colorPosition int = position * 4

	tile.Depth[position] = depth
	tile.Frame[colorPosition+R], tile.Frame[colorPosition+G], tile.Frame[colorPosition+B] = r, g, b
}

func (tile *Tile) Clear(r, g, b byte) {
	for y := tile.Y; y < tile.Y+TileYSize; y++ {
		for x := tile.X; x < tile.X+TileXSize; x++ {
			tile.Set(tile.ConvertPosition(x, y),
				r, g, b, math.MaxFloat32)
		}
	}
}
