package main

type Triangle struct {
	uv [3]Vertex

	vertices [3]Vertex
	normals  [3]Vertex
}

func (triangle Triangle) Multiply(m2 *Matrix) Triangle {
	for i := range triangle.vertices {
		var matrix Matrix = triangle.vertices[i].Matrix()
		matrix = matrix.Multiply(m2)

		triangle.vertices[i] = matrix.Vertex()
	}

	return triangle
}
