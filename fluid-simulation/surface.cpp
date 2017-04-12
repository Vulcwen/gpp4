// Template, major revision 6
// IGAD/NHTV - Jacco Bikker - 2006-2015

// -----------------------------------------------------------
// DISCLAIMER:
// this code is not efficient, may contain memory leaks,
// and could be more readable.
// -----------------------------------------------------------

#include "template.h"

namespace Tmpl8 {

	void NotifyUser(char* s);

#define min(a,b) (((a)<(b))?(a):(b))

	Surface8::Surface8(char* a_File) :
		m_Buffer(NULL),
		m_Width(0), m_Height(0)
	{
		FILE* f = fopen(a_File, "rb");
		if (!f)
		{
			char t[128];
			sprintf(t, "File not found: %s", a_File);
			NotifyUser(t);
			return;
		}
		else fclose(f);
		LoadImage(a_File);
	}

	Surface8::~Surface8()
	{
		FREE64(m_Buffer);
	}

	void Surface8::LoadImage(char* a_File)
	{
		char binFile[1024], *lastDot = binFile + strlen(a_File), *pos = binFile;
		strcpy(binFile, a_File);
		while (strstr(pos + 1, ".")) lastDot = pos = strstr(pos + 1, ".");
		*lastDot = 0;
		strcat(lastDot, ".bin");
		FILE* f = fopen(binFile, "rb");
		if (f)
		{
			fread(&m_Width, 4, 1, f);
			fread(&m_Height, 4, 1, f);
			m_Buffer = (unsigned char*)MALLOC64(m_Width * m_Height);
			fread(m_Buffer, m_Width * m_Height, 1, f);
			for (int i = 0; i < PALETTE_LEVELS; i++)
			{
				palette[i] = new Pixel[256];
				fread(palette[i], 256 * sizeof(Pixel), 1, f);
			}
			fclose(f);
		}
		else
		{
			FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
			fif = FreeImage_GetFileType(a_File, 0);
			if (fif == FIF_UNKNOWN) fif = FreeImage_GetFIFFromFilename(a_File);
			FIBITMAP* tmp = FreeImage_Load(fif, a_File);
			FIBITMAP* t32 = FreeImage_ConvertTo24Bits(tmp);
			FIBITMAP* dib = FreeImage_ColorQuantize(t32, FIQ_NNQUANT); // FIQ_WUQUANT or FIQ_NNQUANT
			FreeImage_Unload(tmp);
			FreeImage_Unload(t32);
			unsigned char* bits = FreeImage_GetBits(dib);
			m_Width = m_Pitch = FreeImage_GetWidth(dib);
			m_Height = FreeImage_GetHeight(dib);
			m_Buffer = (unsigned char*)MALLOC64(m_Width * m_Height);
			for (int y = 0; y < m_Height; y++)
			{
				unsigned char* line = FreeImage_GetScanLine(dib, m_Height - 1 - y);
				memcpy(m_Buffer + y * m_Pitch, line, m_Width);
			}
			RGBQUAD* pal = FreeImage_GetPalette(dib);
			for (int i = 0; i < PALETTE_LEVELS; i++)
			{
				palette[i] = new Pixel[256];
				int scale = (PALETTE_LEVELS * 3) / 4, shift = PALETTE_LEVELS / 3;
				for (int j = 0; j < 256; j++)
				{
					int r = min(255, (pal[j].rgbRed * (i + shift)) / scale);
					int g = min(255, (pal[j].rgbGreen * (i + shift)) / scale);
					int b = min(255, (pal[j].rgbBlue * (i + shift)) / scale);
					palette[i][j] = (r << 16) + (g << 8) + b;
				}
			}
			FreeImage_Unload(dib);
			FILE* f = fopen(binFile, "wb");
			fwrite(&m_Width, 4, 1, f);
			fwrite(&m_Height, 4, 1, f);
			fwrite(m_Buffer, m_Width * m_Height, 1, f);
			for (int i = 0; i < PALETTE_LEVELS; i++) fwrite(palette[i], 256 * sizeof(Pixel), 1, f);
			fclose(f);
		}
	}

	// -----------------------------------------------------------
	// True-color surface class implementation
	// -----------------------------------------------------------

	Surface::Surface(int a_Width, int a_Height, Pixel* a_Buffer, int a_Pitch) :
		m_Width(a_Width),
		m_Height(a_Height),
		m_Buffer(a_Buffer),
		m_Pitch(a_Pitch)
	{
	}

	Surface::Surface(int a_Width, int a_Height) :
		m_Width(a_Width),
		m_Height(a_Height),
		m_Pitch(a_Width)
	{
		m_Buffer = (Pixel*)MALLOC64(a_Width * a_Height * sizeof(Pixel));
	}

	Surface::Surface(char* a_File) :
		m_Buffer(NULL),
		m_Width(0), m_Height(0)
	{
		FILE* f = fopen(a_File, "rb");
		if (!f)
		{
			char t[128];
			sprintf(t, "File not found: %s", a_File);
			NotifyUser(t);
			return;
		}
		else fclose(f);
		LoadImage(a_File);
	}

	void Surface::LoadImage(char* a_File)
	{
		FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
		fif = FreeImage_GetFileType(a_File, 0);
		if (fif == FIF_UNKNOWN) fif = FreeImage_GetFIFFromFilename(a_File);
		FIBITMAP* tmp = FreeImage_Load(fif, a_File);
		FIBITMAP* dib = FreeImage_ConvertTo32Bits(tmp);
		FreeImage_Unload(tmp);
		unsigned char* bits = FreeImage_GetBits(dib);
		m_Width = m_Pitch = FreeImage_GetWidth(dib);
		m_Height = FreeImage_GetHeight(dib);
		m_Buffer = (Pixel*)MALLOC64(m_Width * m_Height * sizeof(Pixel));
		for (int y = 0; y < m_Height; y++)
		{
			unsigned char* line = FreeImage_GetScanLine(dib, m_Height - 1 - y);
			memcpy(m_Buffer + y * m_Pitch, line, m_Width * sizeof(Pixel));
		}
		FreeImage_Unload(dib);
	}

	void Surface::SaveImage(char* a_File)
	{
		FREE_IMAGE_FORMAT fif = FIF_PNG;
		FIBITMAP* bitmap = FreeImage_Allocate(m_Width, m_Height, 24);
		RGBQUAD pixel;
		for (int y = 0; y < m_Height; y++)
			for (int x = 0; x < m_Width; x++)
			{
				Color color = GetAt(x, y);
				pixel.rgbRed = color.r;
				pixel.rgbGreen = color.g;
				pixel.rgbBlue = color.b;
				FreeImage_SetPixelColor(bitmap, x, y, &pixel);
			}
		FreeImage_Save(fif, bitmap, a_File);
		FreeImage_Unload(bitmap);
	}

	Surface::~Surface()
	{
		FREE64(m_Buffer);
	}

	Pixel Surface::GetAt(int x, int y)
	{
		if ((x >= 0) && (y >= 0) && (x < m_Width) && (y < m_Height))
			return *(m_Buffer + y*m_Pitch + x);
		else
			return 0;
	}

	void Surface::Clear(Pixel a_Color)
	{
		int s = m_Width * m_Height;
		for (int i = 0; i < s; i++) m_Buffer[i] = a_Color;
	}

	void Surface::Centre(char* a_String, int y1, Pixel color)
	{
		int x = (m_Width - (int)strlen(a_String) * 6) / 2;
		Print(a_String, x, y1, color);
	}

	void Surface::Print(char* a_String, int x1, int y1, Pixel color)
	{
		Pixel* t = m_Buffer + x1 + y1 * m_Pitch;
		int i;
		for (i = 0; i < (int)(strlen(a_String)); i++)
		{
			long pos = 0;
			if ((a_String[i] >= 'A') && (a_String[i] <= 'Z')) pos = s_Transl[(unsigned short)(a_String[i] - ('A' - 'a'))];
			else pos = s_Transl[(unsigned short)a_String[i]];
			Pixel* a = t;
			char* c = (char*)s_Font[pos];
			int h, v;
			for (v = 0; v < 5; v++)
			{
				for (h = 0; h < 5; h++) if (*c++ == 'o') *(a + h) = color, *(a + h + m_Pitch) = 0;
				a += m_Pitch;
			}
			t += 6;
		}
	}

	void Surface::Resize(Surface* a_Orig)
	{
		Pixel* src = a_Orig->GetBuffer(), *dst = m_Buffer;
		int u, v, owidth = a_Orig->GetWidth(), oheight = a_Orig->GetHeight();
		int dx = (owidth << 10) / m_Width, dy = (oheight << 10) / m_Height;
		for (v = 0; v < m_Height; v++)
		{
			for (u = 0; u < m_Width; u++)
			{
				int su = u * dx, sv = v * dy;
				Pixel* s = src + (su >> 10) + (sv >> 10) * owidth;
				int ufrac = su & 1023, vfrac = sv & 1023;
				int w4 = (ufrac * vfrac) >> 12;
				int w3 = ((1023 - ufrac) * vfrac) >> 12;
				int w2 = (ufrac * (1023 - vfrac)) >> 12;
				int w1 = ((1023 - ufrac) * (1023 - vfrac)) >> 12;
				int x2 = ((su + dx) > ((owidth - 1) << 10)) ? 0 : 1;
				int y2 = ((sv + dy) > ((oheight - 1) << 10)) ? 0 : 1;
				Pixel p1 = *s, p2 = *(s + x2), p3 = *(s + owidth * y2), p4 = *(s + owidth * y2 + x2);
				unsigned int r = (((p1 & REDMASK) * w1 + (p2 & REDMASK) * w2 + (p3 & REDMASK) * w3 + (p4 & REDMASK) * w4) >> 8) & REDMASK;
				unsigned int g = (((p1 & GREENMASK) * w1 + (p2 & GREENMASK) * w2 + (p3 & GREENMASK) * w3 + (p4 & GREENMASK) * w4) >> 8) & GREENMASK;
				unsigned int b = (((p1 & BLUEMASK) * w1 + (p2 & BLUEMASK) * w2 + (p3 & BLUEMASK) * w3 + (p4 & BLUEMASK) * w4) >> 8) & BLUEMASK;
				*(dst + u + v * m_Pitch) = (Pixel)(r + g + b);
			}
		}
	}

	void Surface::Line(float x1, float y1, float x2, float y2, Pixel c)
	{
		if ((x1 < 0) || (y1 < 0) || (x1 >= m_Width) || (y1 >= m_Height) ||
			(x2 < 0) || (y2 < 0) || (x2 >= m_Width) || (y2 >= m_Height))
		{
			return;
		}
		float b = x2 - x1;
		float h = y2 - y1;
		float l = fabsf(b);
		if (fabsf(h) > l) l = fabsf(h);
		int il = (int)l;
		float dx = b / (float)l;
		float dy = h / (float)l;
		for (int i = 0; i <= il; i++)
		{
			*(m_Buffer + (int)x1 + (int)y1 * m_Pitch) = c;
			x1 += dx, y1 += dy;
		}
	}

	void Surface::Plot(int x, int y, Pixel c)
	{
		if ((x >= 0) && (y >= 0) && (x < m_Width) && (y < m_Height)) m_Buffer[x + y * m_Pitch] = c;
	}

	void Surface::Box(int x1, int y1, int x2, int y2, Pixel c)
	{
		Line((float)x1, (float)y1, (float)x2, (float)y1, c);
		Line((float)x2, (float)y1, (float)x2, (float)y2, c);
		Line((float)x1, (float)y2, (float)x2, (float)y2, c);
		Line((float)x1, (float)y1, (float)x1, (float)y2, c);
	}

	void Surface::Bar(int x1, int y1, int x2, int y2, Pixel c)
	{
		x1 = clamp(x1, 0, m_Width - 1); x2 = clamp(x2, 0, m_Width - 1);
		y1 = clamp(y1, 0, m_Height - 1); y2 = clamp(y2, 0, m_Height - 1);
		Pixel* a = x1 + y1 * m_Pitch + m_Buffer;
		for ( int y = y1; y <= y2; y++ )
		{
			for ( int x = 0; x <= (x2 - x1); x++ ) a[x] = c;
			a += m_Pitch;
		}
}

Pixel AlphaBlend(Pixel base_pixel, Pixel new_color, int alpha)
{
	if (alpha > 255)
		return 0xFFFFFF;
	const Pixel base_redblue = base_pixel & 0xFF00FF;
	const Pixel base_green = base_pixel & 0xFF00;
	const Pixel new_redblue = new_color & 0xFF00FF;
	const Pixel new_green = new_color & 0xFF00;
	const Pixel blend_redblue = (new_redblue * alpha) >> 8;
	const Pixel blend_green = (new_green * alpha) >> 8;
	const Pixel blend_redblue2 = (base_redblue * (255 - alpha)) >> 8;
	const Pixel blend_green2 = (base_green * (255 - alpha)) >> 8;
	const Pixel sum_redblue = (blend_redblue2 + blend_redblue) & 0xFF00FF;
	const Pixel sum_green = (blend_green2 + blend_green) & 0xFF00;
	return sum_redblue | sum_green;
}

void Surface::Circle(vec2 center, float r, float alpha_delta, Color color)
{
	for(int y = center.y - r; y < center.y + r; y++)
		for (int x = center.x - r; x < center.x + r; x++)
		{
			if (x < 0 || y < 0 || x >= m_Width || y >= m_Height)
				continue;
			vec2 diff = vec2(x, y) - center;
			float lsq = glm::dot(diff, diff);
			if (lsq <= r*r)
			{
				float ratio = lsq / (r*r);
				unsigned long alpha = 0;
				if(color.a > alpha_delta * ratio)
					alpha = color.a - alpha_delta * ratio;
				
				Pixel val = color.value & 0xFFFFFF;
				Pixel screenVal = this->GetAt(x, y);
				val = AlphaBlend(screenVal, val, alpha);
				this->Plot(x, y, val);
			}
		}
}
//void Surface::Cross(int xc, int yc, Color color)
//{
//	Pixel* b_val = xc + yc * m_Pitch + m_Buffer;
//	*b_val += (color.a * color.value & 0xFFFFFF) >> 8;
//
//	if (xc >= 1)
//	{
//		//left
//		b_val = xc - 1 + yc * m_Pitch + m_Buffer;
//		*b_val += (color.a / 2 * color.value & 0xFFFFFF) >> 8;
//	}
//	if (xc < m_Width - 1)
//	{
//		b_val = xc + 1 + yc * m_Pitch + m_Buffer;
//		*b_val += (color.a / 2 * color.value & 0xFFFFFF) >> 8;
//	}
//	if (yc >= 1)
//	{
//		b_val = xc + (yc - 1) * m_Pitch + m_Buffer;
//		*b_val += (color.a / 2 * color.value & 0xFFFFFF) >> 8;
//	}
//	if (yc < m_Height - 1)
//	{
//		b_val = xc + (yc + 1) * m_Pitch + m_Buffer;
//		*b_val += (color.a / 2 * color.value & 0xFFFFFF) >> 8;
//	}
//
//}

void Surface::CopyTo( Surface* a_Dst, int a_X, int a_Y )
{
	Pixel* dst = a_Dst->GetBuffer();
	Pixel* src = m_Buffer;
	if ((src) && (dst)) 
	{
		int srcwidth = m_Width;
		int srcheight = m_Height;
		int srcpitch = m_Pitch;
		int dstwidth = a_Dst->GetWidth();
		int dstheight = a_Dst->GetHeight();
		int dstpitch = a_Dst->GetPitch();
		if ((srcwidth + a_X) > dstwidth) srcwidth = dstwidth - a_X;
		if ((srcheight + a_Y) > dstheight) srcheight = dstheight - a_Y;
		if (a_X < 0) src -= a_X, srcwidth += a_X, a_X =0;
		if (a_Y < 0) src -= a_Y * srcpitch, srcheight += a_Y, a_Y = 0;
		if ((srcwidth > 0) && (srcheight > 0))
		{
			dst += a_X + dstpitch * a_Y;
			for ( int y = 0; y < srcheight; y++ )
			{
				memcpy( dst, src, srcwidth * 4 );
				dst += dstpitch;
				src += srcpitch;
			}
		}
	}
}

void Surface::BlendCopyTo( Surface* a_Dst, int a_X, int a_Y )
{
	Pixel* dst = a_Dst->GetBuffer();
	Pixel* src = m_Buffer;
	if ((src) && (dst)) 
	{
		int srcwidth = m_Width;
		int srcheight = m_Height;
		int srcpitch = m_Pitch;
		int dstwidth = a_Dst->GetWidth();
		int dstheight = a_Dst->GetHeight();
		int dstpitch = a_Dst->GetPitch();
		if ((srcwidth + a_X) > dstwidth) srcwidth = dstwidth - a_X;
		if ((srcheight + a_Y) > dstheight) srcheight = dstheight - a_Y;
		if (a_X < 0) src -= a_X, srcwidth += a_X, a_X =0;
		if (a_Y < 0) src -= a_Y * srcpitch, srcheight += a_Y, a_Y = 0;
		if ((srcwidth > 0) && (srcheight > 0))
		{
			dst += a_X + dstpitch * a_Y;
			for ( int y = 0; y < srcheight; y++ )
			{
				for ( int x = 0; x < srcwidth; x++ ) dst[x] = AddBlend( dst[x], src[x] );
				dst += dstpitch;
				src += srcpitch;
			}
		}
	}
}

void Surface::SetChar( int c, char* c1, char* c2, char* c3, char* c4, char* c5 )
{
	strcpy( s_Font[c][0], c1 );
	strcpy( s_Font[c][1], c2 );
	strcpy( s_Font[c][2], c3 );
	strcpy( s_Font[c][3], c4 );
	strcpy( s_Font[c][4], c5 );
}

void Surface::InitCharset()
{
	SetChar( 0, ":ooo:", "o:::o", "ooooo", "o:::o", "o:::o" );
	SetChar( 1, "oooo:", "o:::o", "oooo:", "o:::o", "oooo:" );
	SetChar( 2, ":oooo", "o::::", "o::::", "o::::", ":oooo" );
	SetChar( 3, "oooo:", "o:::o", "o:::o", "o:::o", "oooo:" );
	SetChar( 4, "ooooo", "o::::", "oooo:", "o::::", "ooooo" );
	SetChar( 5, "ooooo", "o::::", "ooo::", "o::::", "o::::" );
	SetChar( 6, ":oooo", "o::::", "o:ooo", "o:::o", ":ooo:" );
	SetChar( 7, "o:::o", "o:::o", "ooooo", "o:::o", "o:::o" );
	SetChar( 8, "::o::", "::o::", "::o::", "::o::", "::o::" );
	SetChar( 9, ":::o:", ":::o:", ":::o:", ":::o:", "ooo::" );
	SetChar(10, "o::o:", "o:o::", "oo:::", "o:o::", "o::o:" );
	SetChar(11, "o::::", "o::::", "o::::", "o::::", "ooooo" );
	SetChar(12, "oo:o:", "o:o:o", "o:o:o", "o:::o", "o:::o" );
	SetChar(13, "o:::o", "oo::o", "o:o:o", "o::oo", "o:::o" );
	SetChar(14, ":ooo:", "o:::o", "o:::o", "o:::o", ":ooo:" );
	SetChar(15, "oooo:", "o:::o", "oooo:", "o::::", "o::::" );
	SetChar(16, ":ooo:", "o:::o", "o:::o", "o::oo", ":oooo" );
	SetChar(17, "oooo:", "o:::o", "oooo:", "o:o::", "o::o:" );
	SetChar(18, ":oooo", "o::::", ":ooo:", "::::o", "oooo:" );
	SetChar(19, "ooooo", "::o::", "::o::", "::o::", "::o::" );
	SetChar(20, "o:::o", "o:::o", "o:::o", "o:::o", ":oooo" );
	SetChar(21, "o:::o", "o:::o", ":o:o:", ":o:o:", "::o::" );
	SetChar(22, "o:::o", "o:::o", "o:o:o", "o:o:o", ":o:o:" );
	SetChar(23, "o:::o", ":o:o:", "::o::", ":o:o:", "o:::o" );
	SetChar(24, "o:::o", "o:::o", ":oooo", "::::o", ":ooo:" );
	SetChar(25, "ooooo", ":::o:", "::o::", ":o:::", "ooooo" );
	SetChar(26, ":ooo:", "o::oo", "o:o:o", "oo::o", ":ooo:" );
	SetChar(27, "::o::", ":oo::", "::o::", "::o::", ":ooo:" );
	SetChar(28, ":ooo:", "o:::o", "::oo:", ":o:::", "ooooo" );
	SetChar(29, "oooo:", "::::o", "::oo:", "::::o", "oooo:" );
	SetChar(30, "o::::", "o::o:", "ooooo", ":::o:", ":::o:" );
	SetChar(31, "ooooo", "o::::", "oooo:", "::::o", "oooo:" );
	SetChar(32, ":oooo", "o::::", "oooo:", "o:::o", ":ooo:" );
	SetChar(33, "ooooo", "::::o", ":::o:", "::o::", "::o::" );
	SetChar(34, ":ooo:", "o:::o", ":ooo:", "o:::o", ":ooo:" );
	SetChar(35, ":ooo:", "o:::o", ":oooo", "::::o", ":ooo:" );
	SetChar(36, "::o::", "::o::", "::o::", ":::::", "::o::" );
	SetChar(37, ":ooo:", "::::o", ":::o:", ":::::", "::o::" );
	SetChar(38, ":::::", ":::::", "::o::", ":::::", "::o::" );
	SetChar(39, ":::::", ":::::", ":ooo:", ":::::", ":ooo:" );
	SetChar(40, ":::::", ":::::", ":::::", ":::o:", "::o::" );
	SetChar(41, ":::::", ":::::", ":::::", ":::::", "::o::" );
	SetChar(42, ":::::", ":::::", ":ooo:", ":::::", ":::::" );
	SetChar(43, ":::o:", "::o::", "::o::", "::o::", ":::o:" );
	SetChar(44, "::o::", ":::o:", ":::o:", ":::o:", "::o::" );
	SetChar(45, ":::::", ":::::", ":::::", ":::::", ":::::" );
	SetChar(46, "ooooo", "ooooo", "ooooo", "ooooo", "ooooo" );
	SetChar(47, "::o::", "::o::", ":::::", ":::::", ":::::" ); // Tnx Ferry
	SetChar(48, "o:o:o", ":ooo:", "ooooo", ":ooo:", "o:o:o" );
	SetChar(49, "::::o", ":::o:", "::o::", ":o:::", "o::::" );
	char c[] = "abcdefghijklmnopqrstuvwxyz0123456789!?:=,.-() #'*/";
	int i;
	for ( i = 0; i < 256; i++ ) s_Transl[i] = 45;
	for ( i = 0; i < 50; i++ ) s_Transl[(unsigned char)c[i]] = i;
}

void Surface::ScaleColor( unsigned int a_Scale )
{
	int s = m_Pitch * m_Height;
	for ( int i = 0; i < s; i++ )
	{
		Pixel c = m_Buffer[i];
		unsigned int rb = (((c & (REDMASK|BLUEMASK)) * a_Scale) >> 5) & (REDMASK|BLUEMASK);
		unsigned int g = (((c & GREENMASK) * a_Scale) >> 5) & GREENMASK;
		m_Buffer[i] = rb + g;
	}
}

Sprite::Sprite( Surface* a_Surface, unsigned int a_NumFrames ) :
	m_Width(  a_Surface->GetWidth() / a_NumFrames ),
	m_Height( a_Surface->GetHeight() ),
	m_Pitch(  a_Surface->GetWidth() ),
	m_NumFrames( a_NumFrames ),
	m_CurrentFrame( 0 ),
	m_Flags( 0 ),
	m_Start( new unsigned int*[a_NumFrames] ),
	m_Surface( a_Surface )
{
	InitializeStartData();
}

Sprite::~Sprite()
{
	delete m_Surface;
	for ( unsigned int i = 0; i < m_NumFrames; i++ ) delete m_Start[i];
	delete m_Start;
}

void Sprite::Draw( int a_X, int a_Y, Surface* a_Target )
{
	if ((a_X < -m_Width) || (a_X > (a_Target->GetWidth() + m_Width))) return;
	if ((a_Y < -m_Height) || (a_Y > (a_Target->GetHeight() + m_Height))) return;
	int x1 = a_X, x2 = a_X + m_Width;
	int y1 = a_Y, y2 = a_Y + m_Height;
	Pixel* src = GetBuffer() + m_CurrentFrame * m_Width;
	if (x1 < 0)
	{
		src += -x1;
		x1 = 0;
	}
	if (x2 > a_Target->GetWidth()) x2 = a_Target->GetWidth();
	if (y1 < 0) 
	{ 
		src += -y1 * m_Pitch;
		y1 = 0;
	}
	if (y2 > a_Target->GetHeight()) y2 = a_Target->GetHeight();
	Pixel* dest = a_Target->GetBuffer();
	int xs;
	const int dpitch = a_Target->GetPitch();
	if ((x2 > x1) && (y2 > y1))
	{
		unsigned int addr = y1 * dpitch + x1;
		const int width = x2 - x1;
		const int height = y2 - y1;
		for ( int y = 0; y < height; y++ )
		{
			const int line = y + (y1 - a_Y);
			const int lsx = m_Start[m_CurrentFrame][line] + a_X;
			if (m_Flags & FLARE)
			{
				xs = (lsx > x1)?lsx - x1:0;
				for ( int x = xs; x < width; x++ )
				{
					const Pixel c1 = *(src + x);
					if (c1 & 0xffffff) 
					{
						const Pixel c2 = *(dest + addr + x);
						*(dest + addr + x) = AddBlend( c1, c2 );
					}
				}
			}
			else 
			{
				xs = (lsx > x1)?lsx - x1:0;
				for ( int x = xs; x < width; x++ )
				{
					const Pixel c1 = *(src + x);
					if (c1 & 0xffffff) *(dest + addr + x) = c1;
				}
			}
			addr += dpitch;
			src += m_Pitch;
		}
	}
}

void Sprite::DrawScaled( int a_X, int a_Y, int a_Width, int a_Height, Surface* a_Target )
{
	if ((a_Width == 0) || (a_Height == 0)) return;
	for ( int x = 0; x < a_Width; x++ ) for ( int y = 0; y < a_Height; y++ )
	{
		int u = (int)((float)x * ((float)m_Width / (float)a_Width));
		int v = (int)((float)y * ((float)m_Height / (float)a_Height));
		Pixel color = GetBuffer()[u + v * m_Pitch];
		if (color & 0xffffff) a_Target->GetBuffer()[a_X + x + ((a_Y + y) * a_Target->GetPitch())] = color;
	}
}

void Sprite::InitializeStartData()
{
    for ( unsigned int f = 0; f < m_NumFrames; ++f )
    {
        m_Start[f] = new unsigned int[m_Height];
     	for ( int y = 0; y < m_Height; ++y )
     	{
      	    m_Start[f][y] = m_Width;
			Pixel* addr = GetBuffer() + f * m_Width + y * m_Pitch;
     	    for ( int x = 0; x < m_Width; ++x )
     	    {
                if (addr[x])
     	        {
     	            m_Start[f][y] = x;
                    break;
                }
            }
		}
	}
}

}; // namespace Tmpl8