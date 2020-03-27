
#pragma once

#ifndef IMAGE_HPP
#define IMAGE_HPP

#include<vector>
#include<string>
#include<fstream>

///////////////////////////////////////////////////////////////////////////////////////////////////
//forward declaration
///////////////////////////////////////////////////////////////////////////////////////////////////

template<class T> class Image;
using image = Image<unsigned char>;
using imagef = Image<float>;
using imaged = Image<double>;

///////////////////////////////////////////////////////////////////////////////////////////////////
//Image
///////////////////////////////////////////////////////////////////////////////////////////////////

template<class T> class Image
{
public:

	Image() : m_width(), m_height(), m_data()
	{
	}
	Image(const int width, const int height) : m_width(width), m_height(height), m_data(3 * width * height)
	{
	}

	int width() const
	{
		return m_width;
	}
	int height() const
	{
		return m_height;
	}

	T *operator()(const int x, const int y)
	{
		return &m_data[3 * (x + m_width * y)];
	}
	const T *operator()(const int x, const int y) const
	{
		return &m_data[3 * (x + m_width * y)];
	}

private:
	
	int m_width;
	int m_height;
	std::vector<T> m_data;
};



//save as bitmap
inline void save_as_bmp(const image &img, const std::string &filename)
{
	const int width = img.width();
	const int height = img.height();

	std::ofstream ofs(filename, std::ios::binary);

	const int row_bytes = ((width * 3 + 3) >> 2) << 2;

	const short bfType = 0x4d42;
	ofs.write((char*)&bfType, 2);
	const int bfSize = 14 + 40 + row_bytes * height;
	ofs.write((char*)&bfSize, 4);
	const short bfReserved1 = 0;
	ofs.write((char*)&bfReserved1, 2);
	const short bfReserved2 = 0;
	ofs.write((char*)&bfReserved2, 2);
	const int bfOffBits = 14 + 40;
	ofs.write((char*)&bfOffBits, 4);

	const int biSize = 40;
	ofs.write((char*)&biSize, 4);
	const int biWidth = width;
	ofs.write((char*)&biWidth, 4);
	const int biHeight = height;
	ofs.write((char*)&biHeight, 4);
	const short biPlanes = 1;
	ofs.write((char*)&biPlanes, 2);
	const short biBitCount = 24;
	ofs.write((char*)&biBitCount, 2);
	const int biCompression = 0;
	ofs.write((char*)&biCompression, 4);
	const int biSizeImage = 0;
	ofs.write((char*)&biSizeImage, 4);
	const int biXPelsPerMeter = 0;
	ofs.write((char*)&biXPelsPerMeter, 4);
	const int biYPelsPerMeter = 0;
	ofs.write((char*)&biYPelsPerMeter, 4);
	const int biClrUsed = 0;
	ofs.write((char*)&biClrUsed, 4);
	const int biClrImportant = 0;
	ofs.write((char*)&biClrImportant, 4);

	std::vector<unsigned char> scanline(
		row_bytes
	);
	for(int y = 0; y < height; y++){
		for(int x = 0; x < width; x++){
			scanline[3 * x + 0] = img(x, y)[2]; //r
			scanline[3 * x + 1] = img(x, y)[1]; //g
			scanline[3 * x + 2] = img(x, y)[0]; //b
		}
		ofs.write((char*)scanline.data(), row_bytes);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#endif
