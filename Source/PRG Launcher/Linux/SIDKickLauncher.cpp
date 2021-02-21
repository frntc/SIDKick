#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

unsigned char skBin[ 8 * 1024 * 1024 ];

unsigned char memory[ 8 * 1024 * 1024 ];

unsigned char patDir[ 24 ] = { 'E', 'M', 'P', 'T', 'Y', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char patPRG[ 20 ] = { 'S', 'I', 'D', 'K', 'I', 'C', 'K', ' ', 'R', 'E', 'P', 'O', 0, 0, 0, 0, 0, 0, 0, 0 };

bool compare( unsigned char *a, unsigned char *b, int l )
{
	for ( int i = 0; i < l; i++ )
		if ( *( a++ ) != *( b++ ) )
			return false;
	return true;
}

int main()
{
	printf( "reading SIDKick firmware.\n" );
	FILE *f = fopen( "SIDKick.ino.bin", "rb" );
	if ( f == NULL )
	{
		printf( "error reading .bin!\n" );
		exit( 1 );
	}
	fseek( f, 0, SEEK_END );
	int binSize = ftell( f );
	fseek( f, 0, SEEK_SET );
	fread( skBin, 1, binSize, f );
	fclose( f );

	int ofsDirectory = 0;
	while ( ofsDirectory < binSize - 24 && !compare( &skBin[ ofsDirectory++ ], patDir, 24 ) ) {}
	ofsDirectory--;

	int ofsPRG = 0;
	while ( ofsPRG < binSize - 24 && !compare( &skBin[ ofsPRG++ ], patPRG, 20 ) ) {}
	ofsPRG--;

	//printf( "ofs dir: %d\n", ofsDirectory );
	//printf( "ofs prg: %d\n", ofsPRG );

	int curOffset = 0;
	int nPRGs = 0;

	char *fn, fn2[ 4096 ];
	printf( "\nparsing PRG list:\n" );

	FILE *g = fopen( "prg.lst", "rt" );
	if ( g == NULL )
	{
		printf( "error reading prg.lst!\n" );
		exit( 1 );
	}

	printf( "         menu name : filename\n" );
	while ( !feof( g ) && nPRGs < 16 )
	{
		int printName = -1;
		fgets( fn2, 4095, g );
		fn = fn2;
		for ( int i = 0; i < 4095; i++ )
		{
			if ( fn[ i ] == ':' && printName == -1 )
			{
				printName = i + 1;
				fn[ i ] = 0;
			}
			if ( fn[ i ] == 13 || fn[ i ] == 10 ) fn[ i ] = 0;
		}
	#ifdef WIN32
		strupr( &fn[ printName ] );
	#else
		char *s = &fn[ printName ];
		while (*s) { *s = toupper (*s); s++; }
	#endif

		if ( strlen( fn ) > 0 )
		{
			printf( "%18s : %s\n", &fn[ printName ], fn );

			FILE *h = fopen( fn, "rb" );
			if ( h != NULL )
			{
				fseek( h, 0, SEEK_END );
				int s = ftell( h );
				fseek( h, 0, SEEK_SET );

				fread( &skBin[ ofsPRG ], 1, s, h );
				ofsPRG += s;

				// create directory entry
				fn[ printName + 17 ] = 0;
				memcpy( &skBin[ ofsDirectory ], &fn[ printName ], 18 );

				skBin[ ofsDirectory + 18 ] = 0;
				skBin[ ofsDirectory + 19 ] = ( curOffset >> 16 );
				skBin[ ofsDirectory + 20 ] = ( curOffset >> 8 ) & 255;
				skBin[ ofsDirectory + 21 ] = ( curOffset ) & 255;

				skBin[ ofsDirectory + 22 ] = s >> 8;
				skBin[ ofsDirectory + 23 ] = s & 255;

				ofsDirectory += 24;
				curOffset += s;
				nPRGs++;

				fclose( h );
			} else
			{
				printf( "error reading '%s', skipping...\n", fn );
			}
		}
	}
	fclose( g );

	printf( "\nwriting new SIDKick firmware file.\n" );
	f = fopen( "SIDKick_Launcher.ino.bin", "wb" );
	int written = 0;
	if ( f != NULL )
	{
		written = fwrite( skBin, 1, binSize, f );
		fclose( f );
	}
	if ( f == NULL || written != binSize )
	{
		printf( "error writing .bin\n" );
		exit( 1 );
	}

	return 0;
}
