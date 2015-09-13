#include "Union_find.h"


using namespace std;

bool MergeBlob :: If_Connected( int i , int j){ return Root(i) == Root(j) ;}

int MergeBlob :: Root(int i)
{
	while(i != array_tree[i])
	{
		array_tree[i] = array_tree[array_tree[i]];
		i = array_tree[i];
	}
	return i;
}

void MergeBlob :: Union(int i , int j)
{
	int p = Root(i);
	int q = Root(j);

	if( p == q) return;
	if( array_weight[p] < array_weight[q] )
	{
		array_tree[p] = q;
		array_weight[q] += array_weight[p];
	}
	else
	{
		array_tree[q] = p;
		array_weight[p] += array_weight[q];
	}
	//--blob_number;
}

void MergeBlob :: Union_process()
{
	int i , j;
	int current_position = 0;
	int true_length = image_length / neglect_factor_x;
	int true_height = image_height / neglect_factor_y;

	for( i = 0 ; i < true_height - 1 ; ++i)
	{
		for ( j = 0 ; j < true_length - 1 ; ++j)
		{
			if(array_image[current_position] == array_image[current_position + 1])
				{ Union(current_position , current_position + 1) ;}
			if(array_image[current_position] == array_image[current_position + true_length])
				{ Union(current_position , current_position + true_length) ;}
			++current_position;
		}
		if(array_image[current_position] == array_image[current_position + true_length])
				{ Union(current_position , current_position + true_length);}
		++current_position;
	}
	for( j = 0 ; j < true_length - 1; ++j)
	{
		if(array_image[current_position] == array_image[current_position + 1])
				{ Union(current_position , current_position + 1);}
		++current_position;
	}

    //cout << "blob_number = " << blob_number << endl;
	//Display_array_image();
	//Display_array_tree();
	//Display_array_weight();
}

void MergeBlob :: Calculate_blob()
{
	int i , j;
	int counter_blob = 0;
	int *root_blob_position = new int[valid_length_total];

	int true_length = image_length/neglect_factor_x;
	int true_height = image_height/neglect_factor_y;

	for( i = 0 ; i < valid_length_total; ++i)
	{
        if((array_weight[i] > blob_pixel_number_threshold)&& (i == array_tree[i])) ++blob_number;
	}

	blob = new Blob[blob_number];
	cout << "blob_number ====== " << blob_number << endl;

	for( i = 0 ; i < valid_length_total; ++i)
	{
        root_blob_position[i] = -1;
        if((array_weight[i] > blob_pixel_number_threshold)&& (i == array_tree[i]))
        {
            //cout << "array_weight[i]" << array_weight[i]+1 << endl;
            blob[counter_blob].pixel_index_x = new int[array_weight[i]+1];
            blob[counter_blob].pixel_index_y = new int[array_weight[i]+1];

            blob[counter_blob].pixel_index_x[0] = 0;
            blob[counter_blob].pixel_index_y[0] = 0;

            blob[counter_blob].color = array_image[i];

            root_blob_position[i] = counter_blob++;
        }
	}

	for( i = 0 ; i < valid_length_total ; ++i)
	{
		int father = Root(i);
		int tmp = root_blob_position[father];

        if( tmp != -1)
        {
            blob[tmp].pixel_index_x[++blob[tmp].pixel_index_x[0]] = neglect_factor_x * (i%true_length);
            blob[tmp].pixel_index_y[++blob[tmp].pixel_index_y[0]] = neglect_factor_y * (i/true_length);
        }

	}
	for( i = 0 ; i < counter_blob ; ++i)
	{
		int big_x = blob[i].pixel_index_x[1];
		int big_y = blob[i].pixel_index_y[1];
		int small_x = blob[i].pixel_index_x[1];
		int small_y = blob[i].pixel_index_y[1];

		int sum_x = blob[i].pixel_index_x[1];
		int sum_y = blob[i].pixel_index_y[1];

		for( j = 2 ; j <= blob[i].pixel_index_x[0] ; ++j )
		{
			sum_x += blob[i].pixel_index_x[j];
			sum_y += blob[i].pixel_index_y[j];
			if(blob[i].pixel_index_x[j] > big_x) {big_x = blob[i].pixel_index_x[j] ;}
			else if(blob[i].pixel_index_x[j] < small_x) {small_x = blob[i].pixel_index_x[j] ;}

			if(blob[i].pixel_index_y[j] > big_y) {big_y = blob[i].pixel_index_y[j] ;}
			else if(blob[i].pixel_index_y[j] < small_y) {small_y = blob[i].pixel_index_y[j] ;}
		}

		blob[i].boundingbox_length = big_x - small_x + 1;
		blob[i].boundingbox_height = big_y - small_y + 1;
		blob[i].boundingbox_vertex_x = small_x;
		blob[i].boundingbox_vertex_y = small_y;
		blob[i].centroid_x = sum_x / blob[i].pixel_index_x[0];
		blob[i].centroid_y = sum_y / blob[i].pixel_index_y[0];

		//DisplayBlob(i);
		//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n");
	}
	delete []root_blob_position;

}

void MergeBlob :: DisplayBlob(int tmp)
{
    cout << endl;
    if(tmp >= blob_number || tmp <0 ){ cout << "Out of bound" << endl; }
    else
    {
        cout << "color\t\t\t -> " << blob[tmp].color << endl;
        cout << "centriod\t\t -> (" << blob[tmp].centroid_x << ',' << blob[tmp].centroid_y << ')' << endl;
        cout << "boundingbox_vertex\t -> (" << blob[tmp].boundingbox_vertex_x << ',' << blob[tmp].boundingbox_vertex_y << ')' << endl;
        cout << "boundingbox_length\t -> " << blob[tmp].boundingbox_length << endl;
        cout << "boundingbox_height\t -> " << blob[tmp].boundingbox_height << endl;
        cout << "pixel_number\t\t -> " << blob[tmp].pixel_index_x[0] << endl;
        cout << "detailed pixels\t\t -> \t" ;
        for(int i = 1 ; i <= blob[tmp].pixel_index_x[0] ; ++i){ cout << '(' << blob[tmp].pixel_index_x[i] << ',' << blob[tmp].pixel_index_y[i] << ')' << '\t';}
        cout << endl;
    }
}

void MergeBlob :: Display_array_image()
{
    int i;
    cout << "\n/------------------------------array_image-------------------------------\n";
    for( i = 0 ; i < valid_length_total ; ++i)
    {
        if(i%(image_length/neglect_factor_x) == 0) cout << endl;
        cout << array_image[i] << '\t';
    }
    return ;
}

void MergeBlob :: Display_array_tree()
{
    int i;
    cout << "\n/------------------------------array_tree-------------------------------\n";
    for( i = 0 ; i < valid_length_total ; ++i)
    {
        if(i%(image_length/neglect_factor_x) == 0) cout << endl;
        cout << array_tree[i] << '\t';
    }
    return ;
}

void MergeBlob :: Display_array_weight()
{
    int i;
    cout << "\n/------------------------------array_weight-------------------------------\n";
    for( i = 0 ; i < valid_length_total ; ++i)
    {
        if(i%(image_length/neglect_factor_x) == 0) cout << endl;
        cout << array_weight[i] << '\t';
    }
    return ;
}
