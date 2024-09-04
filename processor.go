package main

import (
	"math"
)

type ProcessedTriangle struct {
	Triangle *Triangle

	Bounds, VS1, VS2 Vertex

	Span float32
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
	var vs1, vs2 Vertex = triangle.Span()

	return ProcessedTriangle{
		triangle,
		triangle.Bounds(), vs1, vs2,
		1 / vs1.CrossProduct(&vs2),
	}
}

func Clip(vertices, uvs, colors []Vertex, component int, direction float32) (clippedVertices, clippedUVs, clippedColors []Vertex) {
	var previousVertex, previousUV, previousColor *Vertex = &vertices[len(vertices)-1], &uvs[len(uvs)-1], &colors[len(colors)-1]
	var previousComponent float32 = direction * (*previousVertex)[component] * .25

	var previousInside bool = previousComponent <= (*previousVertex)[W]

	for index := range vertices {
		var currentComponent float32 = direction * vertices[index][component] * .25
		var currentInside bool = currentComponent <= vertices[index][W]

		if currentInside != previousInside {
			var factor float32 = ((*previousVertex)[W] - previousComponent) /
				(((*previousVertex)[W] - previousComponent) - (vertices[index][W] - currentComponent))

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
		previousComponent = currentComponent
		previousInside = currentInside
	}

	return
}

func BuildAndProcess(triangle *Triangle, tiles *([][]Tile)) {
	var vertices, uvs, colors []Vertex = triangle.Vertices[:], triangle.UV[:], triangle.Color[:]

	for index := range vertices {
		if !vertices[index].InsideClipSpace() {
			for component := X; component <= Y; component++ {
				if len(vertices) > 0 {
					vertices, uvs, colors = Clip(vertices, uvs, colors, component, -1)

					if len(vertices) > 0 {
						vertices, uvs, colors = Clip(vertices, uvs, colors, component, 1)
					} else {
						break
					}

				} else {
					break
				}
			}

			break
		}
	}

	if len(vertices) > 0 {
		vertices[0].Normalize()
		vertices[0].ScreenSpace()

		vertices[1].Normalize()
		vertices[1].ScreenSpace()

		inverseUV, inverseUV1 := 1/vertices[0][W], 1/vertices[1][W]

		t1 := vertices[1].Copy()
		t1.Sub(&vertices[0])

		for index := 0; index < len(vertices)-2; index++ {
			vertices[index+2].Normalize()
			vertices[index+2].ScreenSpace()

			t2 := vertices[index+2].Copy()
			t2.Sub(&vertices[0])

			if t1.Cross(&t2)[Z] < 0 {
				inverseUV2 := 1 / vertices[index+2][W]

				var newTriangle Triangle = Triangle{
					[3]Vertex{
						{uvs[0][X] * inverseUV, uvs[0][Y] * inverseUV, inverseUV},
						{uvs[index+1][X] * inverseUV1, uvs[index+1][Y] * inverseUV1, inverseUV1},
						{uvs[index+2][X] * inverseUV2, uvs[index+2][Y] * inverseUV2, inverseUV2},
					},

					[3]Vertex{vertices[0], vertices[index+1], vertices[index+2]},
					[3]Vertex{colors[0], colors[index+1], colors[index+2]},
					[3]Vertex{vertices[0], vertices[index+1], vertices[index+2]},

					triangle.Texture,
					triangle.Shader,
				}

				inverseUV1, t1 = inverseUV2, t2
				var processedTriangle ProcessedTriangle = Process(&newTriangle)

				var xMin, yMin, xMax, yMax int = processedTriangle.TileBoundary(&Tiles)

				for y := yMin; y < yMax; y++ {
					for x := xMin; x < xMax; x++ {
						Mutex.Lock()
						(*tiles)[x][y].Add(&processedTriangle)
						Mutex.Unlock()
					}
				}
			}
		}
	}
}

func (ts *ProcessedTriangle) TileBoundary(tiles *([][]Tile)) (int, int, int, int) {
	return int(Clamp(float32(math.Floor(float64(ts.Triangle.Bounds()[XMin]/float32(TileXSize)))), 0, len(*tiles))),
		int(Clamp(float32(math.Floor(float64(ts.Triangle.Bounds()[YMin]/float32(TileYSize)))), 0, len((*tiles)[0]))),
		int(Clamp(float32(math.Ceil(float64(ts.Triangle.Bounds()[XMax]/float32(TileXSize)))), 0, len(*tiles))),
		int(Clamp(float32(math.Ceil(float64(ts.Triangle.Bounds()[YMax]/float32(TileYSize)))), 0, len((*tiles)[0])))
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
