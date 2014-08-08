/*
 * cfd.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: dhruvmsaxena
 */
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <sstream>

#include <string>

#include "solver.h"
#include "Source.h"

using namespace std;

/* macros */

#define IX(i,j) ((W+2)*(j)+(i))

/* global variables */

static int W, H, iterations;
static float dt, visc;
char *s_name, *o_name, *output_name;

static float *u, *v, *u_prev, *v_prev;
static int *object_space_grid;

/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/

static void free_data ( void )
{
	if ( u ) free ( u );
	if ( v ) free ( v );
	if ( u_prev ) free ( u_prev );
	if ( v_prev ) free ( v_prev );
	if (object_space_grid) free(object_space_grid);
}

static void clear_data ( void )
{
	int i, size = (W+2)*(H+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = 0.0f;
		object_space_grid[i] = 0;
	}

	//Init objects here
	ifstream file(o_name);
	string line;

	if (file.is_open())
	{
		getline(file, line);
		while (getline(file, line))
		{
			istringstream ss(line);
			string token;

			int edge[4], i = 0;

			while(getline(ss, token, ','))
			{
				edge[i++] = atoi(token.c_str());
			}

			double slope = atan2((edge[3]-edge[1]),(edge[2]-edge[0]));
			double dist = sqrt(pow((edge[3]-edge[1]), 2) + pow((edge[2]-edge[0]), 2));

			for (double x = 1.0; x <= dist; x += 0.1)
			{
				int a, b;
				a = edge[0];
				b = edge[1];
				a += x*cos(slope);
				b += x*sin(slope);

				object_space_grid[IX(a, W-b)] = 1;
			}

			object_space_grid[IX(edge[0], W-edge[1])] = 1;
			object_space_grid[IX(edge[2], W-edge[3])] = 1;
		}
		file.close();
	}
}

static int allocate_data ( void )
{
	int size = (W+2)*(H+2);

	u = (float *) malloc ( size*sizeof(float) );
	v = (float *) malloc ( size*sizeof(float) );
	u_prev = (float *) malloc ( size*sizeof(float) );
	v_prev = (float *) malloc ( size*sizeof(float) );
	object_space_grid = (int *) malloc(size*sizeof(int));

	if (!u || !v || !u_prev || !v_prev || !object_space_grid)
	{
		fprintf ( stderr, "cannot allocate data\n" );
		return ( 0 );
	}

	return ( 1 );
}

static void set_sources ( void )
{
	// Init sources here
	ifstream file(s_name);
	string line;

	getline(file, line);
	int a = atoi(line.c_str());
	Source *sources;
	sources = new Source[a];
	int j = 0;

	if (file.is_open())
	{
		while (getline(file, line))
		{
			istringstream ss(line);
			string token;

			float params[7];
			int i = 0;

			while(getline(ss, token, ','))
			{
				params[i++] = atof(token.c_str());
			}

			if (params[0] == 1)
			{
				params[6] = -1.0f;
			}

			sources[j++] = Source(params);
		}
		file.close();
	}

	for (int i = 0; i < a; i++)
	{
		sources[i].setSource(u, v, W, H);
	}
	delete [] sources;
}

void CFD  ( int width, int height, float timestep, float viscosity, char *obstacles, char *sources, char *output, int iterations = 200 )
{
	int objects;

	W = width;
	H = height;
	dt = timestep;
	visc = viscosity;
	objects = 1;
	o_name = obstacles;
	s_name = sources;

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();

	set_sources();

	for(int i = 0; i < iterations; i++)
	{
		vel_step ( W, H, u, v, u_prev, v_prev, visc, dt, objects, object_space_grid );
	}

	string u_file, v_file;
	u_file = string(output) + "_HorVel.csv";
	v_file = string(output) + "_VertVel.csv";

	ofstream file1(u_file.c_str()), file2(v_file.c_str());

	int i, j;
	if (file1.is_open() && file2.is_open())
	{
		for (i = 0; i < W+2; i++)
		{
			for (j = H+1; j > 0; j--)
			{
				file1 << u[IX(i,j)] << ',';
				file2 << v[IX(i,j)] << ',';
			}
			file1 << u[IX(i,j)] << '\n';
			file2 << v[IX(i,j)] << '\n';
		}
		file1.close();
		file2.close();
	}

	free_data();

	exit ( 0 );
}

int main ( int argc, char ** argv )
{
	int objects;

	if ( argc != 1 && (argc < 6 || argc > 8)) {
		fprintf ( stderr, "usage : %s W H dt visc objects sources output iterations\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t W      : grid width\n" );
		fprintf ( stderr, "\t H      : grid height\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t objects : file with data about objects in the map\n" );
		fprintf ( stderr, "\t sources : file with data about sources in the map\n" );
		fprintf ( stderr, "\t output  : name of the output files \n");
		fprintf ( stderr, "\t iterations : number of iterations to run simulation for\n" );
		exit ( 1 );
	}

	if ( argc == 1 )
	{
		W = 500;
		H = 500;
		dt = 0.1f;
		visc = 0.0f;
	}

	else
	{
		W = atoi(argv[1]);
		H = atoi(argv[2]);
		dt = atof(argv[3]);
		visc = atof(argv[4]);
		objects = 1;

		if (argc == 6)
		{
			o_name = argv[5];
		}

		else if (argc == 7)
		{
			o_name = argv[5];
			s_name = argv[6];
		}

		else if (argc == 8)
		{
			o_name = argv[5];
			s_name = argv[6];
			output_name = argv[7];
		}

		else
		{
			o_name = argv[5];
			s_name = argv[6];
			output_name = argv[7];
			iterations = atoi(argv[8]);
		}
	}

	//CFD(N, u, v, u_prev, v_prev, visc, dt, objects, object_space_grid, iterations);
	 CFD(W, H, dt, visc, o_name, s_name,  output_name, iterations=200);
}
