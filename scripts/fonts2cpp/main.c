
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <VFlib-3_6.h>



int main(int argc, char **argv)
{
    //char * lib_file = "vflibcap.pk";
    char * lib_file = "/etc/vflib3/vflibcap";

    if (argc!=5)
    {
        printf("Usage: fonts2cpp <font_name> <font_height> <out_header_prefix> <0:Latin only/1:Latin+Asian>\n");
        return -1;
    }

    // Get parameters:
    char *font_name= argv[1];
    int FONT_SIZE = atoi(argv[2]);
    const char *out_header= argv[3];
    const int include_asian = atoi(argv[4]);

    // init lib:
    int ret = VF_Init(lib_file,NULL);
    if (ret)
    {
        printf("error opening fonts file\n");
        return -1;
    }

    //int font_id = VF_OpenFont2( "helvR12.bdf", FONT_SIZE, 1,1);
    int font_id = VF_OpenFont2(font_name , FONT_SIZE, 1,1);
    if (font_id<0)
    {
        printf("error opening font\n");
        return -1;
    }


    char header_fil[1000];
    sprintf(header_fil,"mrpt_font_%s.h",out_header);

    FILE *f_header = fopen(header_fil,"wt");
    assert(f_header);

    fprintf(f_header,"/* MRPT font header  \n");
    fprintf(f_header," *  Input font file: %s\n", font_name);
    fprintf(f_header," *  File generated automatically by fonts2cpp utility\n");
    fprintf(f_header," *  Jose Luis Blanco (C) 2008                         */\n\n");
    fprintf(f_header,"const uint32_t mrpt_font_%s [] = {\n",out_header);


    // Test character:
    int bmp_fixed_w,bmp_fixed_h;
    {
        VF_BITMAP  bmp = VF_GetBitmap2(font_id, 'A', 1.0, 1.0);
        assert(bmp);
        bmp_fixed_w = bmp->bbx_width;
        assert(bmp_fixed_w>0 && bmp_fixed_w<=32);

        bmp_fixed_h = bmp->bbx_height;
        assert(bmp_fixed_h==FONT_SIZE);
    }
    fprintf(f_header,"%i,%i, /* width, height */\n",bmp_fixed_w,bmp_fixed_h );


    // The sets of UNICODE characters:
    long  char_sets_latin []        = { 0x0000,0x00ff, 0,0 };
    long  char_sets_latin_asian []  = { 0x0000,0x00ff, 0x3000,0x312f, 0x4E00,0x9fff, 0,0 };

    long *char_sets = include_asian ? char_sets_latin_asian : char_sets_latin;

    long char_ini,char_end;

    // Go over all the sets of characters:
    do
    {
        char_ini = *char_sets++;
        char_end = *char_sets++;

        long  character;

        if (char_end) printf("Generating char range: 0x%04lX-0x%04lX...\n",char_ini,char_end);

        fprintf(f_header,"0x%04lX,0x%04lX%c /* %s */\n",char_ini,char_end, char_end ? ',':' ', char_end ? "UNICODE characters range:" : "END FLAG" );

        if (char_end)
        for (character=char_ini;character<=char_end;character++)
        {
            VF_BITMAP  bmp = VF_GetBitmap2(font_id, character, 1.0, 1.0);
            if (bmp)
            {
                int  bmp_w = bmp->bbx_width;
                assert(bmp_w==bmp_fixed_w);

                int  bmp_h = bmp->bbx_height;
                assert(bmp_h==bmp_fixed_h);

                {
                    FILE *f=fopen("out.txt","w");
                    assert(f);
                    VF_ImageOut_ASCIIArt(bmp,f,bmp_w,bmp_h, 0,0, 0,0, 0,0,0,1);
                    fclose(f);
                }

                // Process text file and convert in a bit-map:
                {
                    char buf[50];
                    int N;
                    FILE *f=fopen("out.txt","r");
                    assert(f);

                    int y,x;

                    for (y=0;y<bmp_h;y++)
                    {
                        N=fread(buf,1,bmp_w,f);
                        assert(N==bmp_w);

                        // Convert into bits:
                        int    bits=0;
                        for (x=0;x<bmp_w;x++)
                            if (buf[x]!=' ')
                                bits |= 1 << x;

                        fprintf(f_header,"%i,",bits);
                        //buf[bmp_w]=0; printf("%s : %08X\n",buf,bits);

                        fread(buf,1,1,f);   // skip '\n'
                    }
                    fprintf(f_header,"\n");

                    fclose(f);
                }

                VF_FreeBitmap(bmp);
            }
            else
            {
                // We dont have that char!
                int y;
                for (y=0;y<bmp_fixed_h;y++)
                    fprintf(f_header,"0,");
                fprintf(f_header,"\n");
            }
        } // end char segment:

    } while ( char_end>0 );
     // end for each char segment

    fprintf(f_header,"};\n\n");


    // Close font:
    VF_CloseFont(font_id);

    fclose(f_header);

	return 0;
}



