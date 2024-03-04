package main

type VertexShader func(vertex *Vertex, matrices ...*Matrix)
type FragmentShader func(r, g, b *float32)

type Shader struct {
	Vertex VertexShader

	Fragment FragmentShader
}