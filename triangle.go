package main

type Triangle struct {
	UV [3]Vertex

	Vertices [3]Vertex
	Normals  [3]Vertex
}

func (triangle *Triangle) Transform(m2 *Matrix) {
	for index := range triangle.Vertices {
		var matrix Matrix = triangle.Vertices[index].Matrix()
		matrix = matrix.Multiply(m2)

		triangle.Vertices[index] = matrix.Vertex()
	}
}

func (triangle *Triangle) ScreenSpace() {
	for index := range triangle.Vertices {
		triangle.Vertices[index].ScreenSpace()
	}
}

func (triangle *Triangle) Sort() {
	for i := range triangle.Vertices {
		for j := i + 1; j < len(triangle.Vertices); j++ {
			if triangle.Vertices[i][Y] > triangle.Vertices[j][Y] {

				triangle.Vertices[i].Swap(&triangle.Vertices[j])
				triangle.Normals[i].Swap(&triangle.Normals[j])
				triangle.UV[i].Swap(&triangle.UV[j])
			}
		}
	}
}

func (triangle *Triangle) Span() (Vertex, Vertex) {
	return Vertex{triangle.Vertices[1][X] - triangle.Vertices[0][X], triangle.Vertices[1][Y] - triangle.Vertices[0][Y]},
		Vertex{triangle.Vertices[2][X] - triangle.Vertices[0][X], triangle.Vertices[2][Y] - triangle.Vertices[0][Y]}
}

func (triangle *Triangle) Barycentric(vs1, vs2 *Vertex, span float32, x, y int) (float32, float32, float32) {
	var q Vertex = Vertex{float32(x) - triangle.Vertices[0][X], float32(y) - triangle.Vertices[0][Y]}
	span = 1 / span

	var s float32 = q.CrossProduct(vs2) * span
	var t float32 = vs1.CrossProduct(&q) * span

	return s, t, 1 - s - t
}
