#include "union_find.h"
#include <ctime>
#include <cstdlib>

const int image_length = 40;
const int image_height = 30;
const int neg_factor_x = 1 << 2;
const int neg_factor_y = 1 << 1;

int main()
{
    srand(time(NULL));

//------------------------------------Generate the image--------------------------------------
    int i , j;
    int tmp;
    int counter = 0;

    unsigned char *image = new unsigned char[image_length*image_height];

    for( i = 0 ; i < image_height ; ++i)
    {
        for( j = 0 ; j < image_length ; ++j)
        {
            if( i%neg_factor_y || j%neg_factor_x ){ image[counter++] = 0; }
            else
            {
                tmp = rand() * 3 / (RAND_MAX + 1);
                image[counter++] = tmp + 48 ;
                //if( i < image_height/2 && j < image_length/2){ fout0 << 1;}
                //else if( i < image_height/2 && j >= image_length/2){ fout0 << 2;}
                //else if( i >= image_height/2 && j < image_length/2){ fout0 << 3;}
                //else if( i >= image_height/2 && j >= image_length/2){ fout0 << 4;}
            }
        }
    }


//-------------------The interface of image and MergeBlob------------------------------------
    MergeBlob mergeblob(image, image_length , image_height , neg_factor_x , neg_factor_y);
//-------------------------------------------------------------------------------------------
    //mergeblob.DisplayBlob(0);
    //mergeblob.Display_array_image();
    //cout << '\n' << "end" << endl;
    delete []image;
    return 0;
}
/*
{
    ofstream fout0("image.txt");
    ifstream fin1("image.txt");

    srand(time(NULL));

    int i , j;
    int tmp;

    unsigned char *image = new unsigned char[image_length*image_height];

    for( i = 0 ; i < image_height ; ++i)
    {
        for( j = 0 ; j < image_length ; ++j)
        {
            if( i%neg_factor_y || j%neg_factor_x ){ fout0 << 0; }
            else
            {
                tmp = rand() * 3 / (RAND_MAX + 1);
                fout0 << tmp ;
                //if( i < image_height/2 && j < image_length/2){ fout0 << 1;}
                //else if( i < image_height/2 && j >= image_length/2){ fout0 << 2;}
                //else if( i >= image_height/2 && j < image_length/2){ fout0 << 3;}
                //else if( i >= image_height/2 && j >= image_length/2){ fout0 << 4;}
            }
        }
        fout0 << endl;
    }

    int counter = 0;
    while(fin1 >> image[counter++]){}
    //while(fin1 >> image[counter]) {cout << image[counter++];}
    //cout << '\n' << counter << endl;

    MergeBlob mergeblob(image, image_length , image_height , neg_factor_x , neg_factor_y);
    //mergeblob.DisplayBlob(0);
    //mergeblob.Display_array_image();

    //cout << '\n' << "end" << endl;

    delete []image;

    return 0;
}
*/
