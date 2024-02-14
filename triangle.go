package main

type Triangle struct {
	uv [3]Vertex

	vertices [3]Vertex
	normals  [3]Vertex
}

func (triangle Triangle) Multiply(m2 *Matrix) Triangle {
	for index := range triangle.vertices {
		var matrix Matrix = triangle.vertices[index].Matrix()
		matrix = matrix.Multiply(m2)

		triangle.vertices[index] = matrix.Vertex()
	}

	return triangle
}

func (triangle Triangle) ScreenSpace() Triangle {
	for index := range triangle.vertices {
		triangle.vertices[index] = triangle.vertices[index].ScreenSpace()
	}

	return triangle
}

func (triangle Triangle) Sort() Triangle {
	for i := range triangle.vertices {
		for j := i + 1; j < len(triangle.vertices); j++ {
			if triangle.vertices[i][Y] > triangle.vertices[j][Y] {

				triangle.vertices[i].Swap(&triangle.vertices[j])
				triangle.normals[i].Swap(&triangle.normals[j])
				triangle.uv[i].Swap(&triangle.uv[j])
			}
		}
	}

	return triangle
}

func (triangle *Triangle) Span() (Vertex, Vertex) {
	return Vertex{triangle.vertices[1][X] - triangle.vertices[0][X], triangle.vertices[1][Y] - triangle.vertices[0][Y]},
		Vertex{triangle.vertices[2][X] - triangle.vertices[0][X], triangle.vertices[2][Y] - triangle.vertices[0][Y]}
}

func (triangle *Triangle) Barycentric(vs1, vs2 *Vertex, x, y *int, span *float32) (float32, float32, float32) {
	var q Vertex = Vertex{float32(*x) - triangle.vertices[0][X], float32(*y) - triangle.vertices[0][Y]}

	var s float32 = q.CrossProduct(vs2) / *span
	var t float32 = vs1.CrossProduct(&q) / *span

	return s, t, 1 - s - t
}
