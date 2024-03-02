package main

import (
	"math"
)

type ProcessedTriangle struct {
	Triangle *Triangle

	Bounds, VS1, VS2 Vertex

	Span, Split float32
}

func Clamp(value float32, min, max int) float32 {
	if value < float32(min) {
		return float32(min)
	} else if value > float32(max) {
		return float32(max)
	}

	return value
}

func Process(triangle *Triangle) ProcessedTriangle {
	var split float32 = triangle.Vertices[0][X] + ((triangle.Vertices[1][Y]-triangle.Vertices[0][Y])/
		(triangle.Vertices[2][Y]-triangle.Vertices[0][Y]))*
		(triangle.Vertices[2][X]-triangle.Vertices[0][X])

	var vs1, vs2 Vertex = triangle.Span()

	return ProcessedTriangle{
		triangle,
		triangle.Bounds(), vs1, vs2,
		1 / vs1.CrossProduct(&vs2), split,
	}
}

func Clip(vertices, uvs, colors []Vertex, component, direction int) (clippedVertices, clippedUVs, clippedColors []Vertex) {
	var previousVertex, previousUV, previousColor *Vertex = &vertices[len(vertices)-1], &uvs[len(uvs)-1], &colors[len(colors)-1]
	var previousInside bool = (*previousVertex)[component] <= (*previousVertex)[W]

	for index := range vertices {
		var currentInside bool = vertices[index][component] <= vertices[index][W]

		if currentInside != previousInside {
			var factor float32 = ((*previousVertex)[W] - (*previousVertex)[component]) /
				(((*previousVertex)[W] - (*previousVertex)[component]) - (vertices[index][W] - vertices[index][component]))

			var copiedVertex, copiedUV, copiedColor Vertex = previousVertex.Copy(), previousUV.Copy(), previousColor.Copy()
			copiedVertex.Interpolate(&vertices[index], factor)
			copiedUV.Interpolate(&uvs[index], factor)
			copiedColor.Interpolate(&colors[index], factor)

			clippedVertices, clippedUVs, clippedColors = append(clippedVertices, copiedVertex),
				append(clippedUVs, copiedUV), append(clippedColors, copiedColor)
		}

		if currentInside {
			clippedVertices, clippedUVs, clippedColors = append(clippedVertices, vertices[index]),
				append(clippedUVs, uvs[index]), append(clippedColors, colors[index])
		}

		previousVertex, previousUV, previousColor = &vertices[index], &uvs[index], &colors[index]
		previousInside = currentInside
	}

	return
}

func BuildAndProcess(triangle *Triangle) (processedTriangles []ProcessedTriangle) {
	var vertices, uvs, colors []Vertex = triangle.Vertices[:], triangle.UV[:], triangle.Color[:]

	for component := X; component <= Y; component++ {
		if len(vertices) > 0 {
			vertices, uvs, colors = Clip(vertices, uvs, colors, component, 1)
		} else {
			break
		}
	}

	if len(vertices) > 0 {
		vertices[0].Normalize()
		vertices[0].ScreenSpace()

		vertices[1].Normalize()
		vertices[1].ScreenSpace()

		for index := 0; index < len(vertices)-2; index++ {
			vertices[index+2].Normalize()
			vertices[index+2].ScreenSpace()

			var newTriangle Triangle = Triangle{
				[3]Vertex{uvs[0], uvs[index+1], uvs[index+2]},
				[3]Vertex{vertices[0], vertices[index+1], vertices[index+2]},
				[3]Vertex{colors[0], colors[index+1], colors[index+2]},
				[3]Vertex{vertices[0], vertices[index+1], vertices[index+2]},

				triangle.Shader,
			}

			processedTriangles = append(processedTriangles, Process(&newTriangle))
		}
	}

	return
}

func (ts *ProcessedTriangle) TileBoundary(tiles *([4][3]Tile)) (int, int, int, int) {
	return int(Clamp(float32(math.Floor(float64(ts.Triangle.Bounds()[XMIN]/float32(TileXSize)))), 0, 4)),
		int(Clamp(float32(math.Floor(float64(ts.Triangle.Bounds()[YMIN]/float32(TileYSize)))), 0, 3)),
		int(Clamp(float32(math.Ceil(float64(ts.Triangle.Bounds()[XMAX]/float32(TileXSize)))), 0, 4)),
		int(Clamp(float32(math.Ceil(float64(ts.Triangle.Bounds()[YMAX]/float32(TileYSize)))), 0, 3))
}

func (ts *ProcessedTriangle) Barycentric(x, y int) (float32, float32, float32) {
	var q Vertex = Vertex{float32(x) - ts.Triangle.Vertices[0][X], float32(y) - ts.Triangle.Vertices[0][Y]}

	var s float32 = q.CrossProduct(&ts.VS2) * ts.Span
	var t float32 = ts.VS1.CrossProduct(&q) * ts.Span

	return 1 - s - t, s, t
}

func (ts *ProcessedTriangle) Inside(x, y int) (bool, float32, float32, float32) {
	var s, t, w float32 = ts.Barycentric(x, y)

	return (s >= 0 && t >= 0 && s+t <= 1),
		s, t, w
}
