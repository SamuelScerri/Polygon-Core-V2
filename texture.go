package ghetty

import (
	"image"
	"log"
	"math"
	"os"
)

const (
	R          = 0
	G          = 1
	B          = 2
	A          = 3
	RGBToFloat = 1. / 255.
)

type Texture struct {
	Width, Height int

	Data []byte
}

func LoadTexture(directory string) Texture {
	file, err := os.Open(directory)

	if err != nil {
		log.Fatal(err)
	}

	defer file.Close()

	image, _, err := image.Decode(file)

	if err != nil {
		log.Fatal(err)
	}

	var texture Texture = Texture{Width: image.Bounds().Dx(), Height: image.Bounds().Dy()}

	for y := 0; y < texture.Height; y++ {
		for x := 0; x < texture.Width; x++ {
			r, g, b, a := image.At(x, y).RGBA()
			texture.Data = append(texture.Data, byte(r), byte(g), byte(b), byte(a))
		}
	}

	return texture
}

func (texture *Texture) ConvertPosition(uv *Vertex) int {
	var tx int = int((*uv)[X] * float32(texture.Width))
	var ty int = int((1 - (*uv)[Y]) * float32(texture.Height))

	return int(math.Abs(float64((ty*texture.Width+tx)*4))) % len(texture.Data)
}

func (texture *Texture) Get(position int) (r, g, b, a float32) {
	return float32(texture.Data[position+R]) * RGBToFloat,
		float32(texture.Data[position+G]) * RGBToFloat,
		float32(texture.Data[position+B]) * RGBToFloat,
		float32(texture.Data[position+A]) * RGBToFloat
}
