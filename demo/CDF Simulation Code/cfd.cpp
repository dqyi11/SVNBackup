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

#define IX(i,j) ((i)+(N+2)*(j))
#define IXRed(i,j) ((i)+(N/sizeDown+2)*(j))

/* global variables */

static int N;
static float dt, visc;
char *s_name, *o_name;
int sizeDown, iterations;

static float * u, * v, * u_prev, * v_prev;
static float *uRed, *vRed, *SAT;
static int * object_space_grid, *objectRed;

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
	if (objectRed) free(objectRed);
	if (uRed) free(uRed);
	if (vRed) free(vRed);
	if (SAT) free(SAT);
}

static void clear_data ( void )
{
	int i, j, size = (N+2)*(N+2), redSize = (N/sizeDown+2)*(N/sizeDown+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = SAT[i] = 0.0f;
		object_space_grid[i] = 0;
	}

	for ( j = 0 ; j < redSize ; j++ )
	{
		uRed[j] = vRed[j] = 0.0f;
		objectRed[j] = 0;
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

			int edge[4], edgeRed[4], i = 0;

			while(getline(ss, token, ','))
			{
				edge[i++] = atoi(token.c_str());
				edgeRed[i-1] = edge[i-1]/sizeDown;
			}

			double slope = atan2((edge[3]-edge[1]),(edge[2]-edge[0]));
			double dist = sqrt(pow((edge[3]-edge[1]), 2) + pow((edge[2]-edge[0]), 2));

			double slopeRed = atan2((edgeRed[3]-edgeRed[1]),(edgeRed[2]-edgeRed[0]));
			double distRed = sqrt(pow((edgeRed[3]-edgeRed[1]), 2) + pow((edgeRed[2]-edgeRed[0]), 2));

			for (double x = 1.0; x <= dist; x += 0.1)
			{
				int a, b;
				a = edge[0];
				b = edge[1];
				a += x*cos(slope);
				b += x*sin(slope);

				object_space_grid[IX(a, b)] = 1;
			}

			for (double x = 1.0; x <= distRed; x += 0.1)
			{
				int a, b;
				a = edgeRed[0];
				b = edgeRed[1];
				a += x*cos(slopeRed);
				b += x*sin(slopeRed);

				objectRed[IXRed(a, b)] = 1;
			}

			objectRed[IXRed(edgeRed[0], edgeRed[1])] = 1;
			objectRed[IXRed(edgeRed[2], edgeRed[3])] = 1;

			object_space_grid[IX(edge[0], edge[1])] = 1;
			object_space_grid[IX(edge[2], edge[3])] = 1;
		}
		file.close();
	}
}

static int allocate_data ( void )
{
	if(!sizeDown) sizeDown = 1;
	int size = (N+2)*(N+2), redSize = (N/sizeDown+2)*(N/sizeDown+2);

	u			= (float *) malloc ( size*sizeof(float) );
	v			= (float *) malloc ( size*sizeof(float) );
	u_prev		= (float *) malloc ( size*sizeof(float) );
	v_prev		= (float *) malloc ( size*sizeof(float) );
	object_space_grid = (int *) malloc(size*sizeof(int));
	objectRed = (int *) malloc(redSize*sizeof(int));
	uRed	= (float *) malloc ( redSize*sizeof(float) );
	vRed	= (float *) malloc ( redSize*sizeof(float) );
	SAT = (float *) malloc(size*sizeof(float));

	if (!u || !v || !u_prev || !v_prev || !object_space_grid ||
			!objectRed || !uRed || !vRed || !SAT)
	{
		fprintf ( stderr, "cannot allocate data\n" );
		return ( 0 );
	}

	return ( 1 );
}

/*
  ----------------------------------------------------------------------
   size reduction
  ----------------------------------------------------------------------
*/

static void reduce(float *x, float *xRed, int b) // x = (N+2)*(N+2), xRed = (N/sizeDown+2)*(N/sizeDown+2)
{
	int NRed = N/sizeDown;
	for (int i = 1; i <= N; i++)
	{
		for (int j = 1; j <= N; j++)
		{
			// Creating the Summed Area Table
			if (i == 1 && j != 1)
			{
				SAT[IX(i,j)] = x[IX(i,j)] + x[IX(i,j-1)];
			}

			else if (j == 1 && i != 1)
			{
				SAT[IX(i,j)] = x[IX(i,j)] + x[IX(i-1,j)];
			}

			else if (i != 1 && j != 1)
			{
				SAT[IX(i,j)] = x[IX(i,j)] + SAT[IX(i-1,j)] + SAT[IX(i,j-1)] - SAT[IX(i-1,j-1)];
			}

			// Averaging over submatrices to reduce size
			if ((i % sizeDown == 0) && (j % sizeDown == 0))
			{
				int h = (i/sizeDown), k = (j/sizeDown);

				if (h == 1 && k == 1)
				{
					xRed[IXRed(h, k)] = SAT[IX(h*sizeDown, k*sizeDown)]/(sizeDown*sizeDown);
				}

				else if (h == 1 && k != 1)
				{
					xRed[IXRed(h, k)] = (SAT[IX(h*sizeDown, k*sizeDown)] - SAT[IX(h*sizeDown, (k-1)*sizeDown)])/(sizeDown*sizeDown);;
				}

				else if (k == 1 && h != 1)
				{
					xRed[IXRed(h, k)] = (SAT[IX(h*sizeDown, k*sizeDown)] - SAT[IX((h-1)*sizeDown, k*sizeDown)])/(sizeDown*sizeDown);
				}

				else
				{
					xRed[IXRed(h, k)] = (SAT[IX(h*sizeDown, k*sizeDown)] - SAT[IX((h-1)*sizeDown, k*sizeDown)] - SAT[IX(h*sizeDown, (k-1)*sizeDown)] + SAT[IX((h-1)*sizeDown, (k-1)*sizeDown)])/(sizeDown*sizeDown);
				}
			}
		}
	}

	// Set boundaries for xRed
	for (int a = 1 ; a <= NRed ; a++)
	{
		xRed[IXRed(0, a)] = b==1 ? -xRed[IXRed(1, a)] : xRed[IXRed(1, a)];
		xRed[IXRed(NRed+1, a)] = xRed[IXRed(NRed, a)];
		xRed[IXRed(a, 0)] = b==2 ? -xRed[IXRed(a, 1)] : xRed[IXRed(a, 1)];
		xRed[IXRed(a, NRed+1)] = b==2 ? -xRed[IXRed(a, NRed)] : xRed[IXRed(a, NRed)];
	}
	xRed[IXRed(0  ,0  )] = 0.5f*(xRed[IXRed(1,0  )]+xRed[IXRed(0  ,1)]);
	xRed[IXRed(0  ,NRed+1)] = 0.5f*(xRed[IXRed(1,NRed+1)]+xRed[IXRed(0  ,NRed)]);
	xRed[IXRed(NRed+1,0  )] = 0.5f*(xRed[IXRed(NRed,0  )]+xRed[IXRed(NRed+1,1)]);
	xRed[IXRed(NRed+1,NRed+1)] = 0.5f*(xRed[IXRed(NRed,NRed+1)]+xRed[IXRed(NRed+1,NRed)]);
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
		sources[i].setSource(u, v, N);
	}
	delete [] sources;
}

/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	int objects;

	if ( argc != 1 && (argc < 4 || argc > 8)) {
		fprintf ( stderr, "usage : %s N dt visc objects sources sizeDown iterations\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t objects : file with data about objects in the map\n" );
		fprintf ( stderr, "\t sources : file with data about sources in the map\n" );
		fprintf ( stderr, "\t sizeDown : numerical factor to reduce resolution, or the square submatrix size\n" );
		fprintf ( stderr, "\t iterations : number of iterations to run simulation for\n" );
		exit ( 1 );
	}

	if ( argc == 1 )
	{
		N = 64;
		dt = 0.1f;
		visc = 0.0f;
		fprintf (stderr, "Using defaults : N=%d dt=%g visc=%g\n", N, dt, visc);
	}

	else
	{
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		visc = atof(argv[3]);
		objects = 1;

		if (argc == 5)
		{
			o_name = argv[4];
		}

		else if (argc == 6)
		{
			o_name = argv[4];
			s_name = argv[5];
			sizeDown = 0;
		}

		else if (argc == 7)
		{
			o_name = argv[4];
			s_name = argv[5];
			sizeDown = atoi(argv[6]);
		}

		else
		{
			o_name = argv[4];
			s_name = argv[5];
			sizeDown = atoi(argv[6]);
			iterations = atoi(argv[7]);
		}
	}

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();

	set_sources();

	for(int i = 0; i < iterations; i++)
	{
		vel_step ( N, u, v, u_prev, v_prev, visc, dt, objects, object_space_grid );
	}

	reduce(u, uRed, 1);
	reduce(v, vRed, 2);

	//WRITE OUT THE REQUIRED MATRICES OUT OF u, v, uRed, vRed

	free_data();

	exit ( 0 );
}



