package main

import (
	"image"
	"log"
	"os"
	"math"
)

const (
	R = 0
	G = 1
	B = 2
	A = 3
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

func (texture *Texture) ConvertPosition(x, y int) int {
	return int(math.Abs(float64((y*texture.Width + x) * 4))) % len(texture.Data)
}

func (texture *Texture) Get(position int) (r, g, b, a byte) {
	return texture.Data[position+R],
		texture.Data[position+G],
		texture.Data[position+B],
		texture.Data[position+A]
}
