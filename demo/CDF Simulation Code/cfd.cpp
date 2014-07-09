/*
 * cfd.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: dhruvmsaxena
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <GL/glut.h>
#include <string>
#include <vector>

#include "solver.h"
#include "Source.h"

using namespace std;
//using namespace std::vector;

/* macros */

#define IX(i,j) ((i)+(N+2)*(j))
#define IXRed(i,j) ((i)+(N/sizeDown+2)*(j))

/* global variables */

static int N;
static float dt, diff, visc;
static float force, source;
static int dvel, objects, lowRes;
char *s_name, *o_name;
int sizeDown;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static float *uRed, *vRed, *densRed, *SAT;
static int * object_space_grid, *objectRed;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;

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
	if ( dens ) free ( dens );
	if ( dens_prev ) free ( dens_prev );
	if (object_space_grid) free(object_space_grid);
	if (objectRed) free(objectRed);
	if (uRed) free(uRed);
	if (vRed) free(vRed);
	if (densRed) free(densRed);
	if (SAT) free(SAT);
}

static void clear_data ( void )
{
	int i, j, size = (N+2)*(N+2), redSize = (N/sizeDown+2)*(N/sizeDown+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = SAT[i] = 0.0f;
		object_space_grid[i] = 0;
	}

	for ( j = 0 ; j < redSize ; j++ )
	{
		uRed[j] = vRed[j] = densRed[j] = 0.0f;
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

	Swarm robots;
}

static int allocate_data ( void )
{
	if(!sizeDown) sizeDown = 1;
	int size = (N+2)*(N+2), redSize = (N/sizeDown+2)*(N/sizeDown+2);

	u			= (float *) malloc ( size*sizeof(float) );
	v			= (float *) malloc ( size*sizeof(float) );
	u_prev		= (float *) malloc ( size*sizeof(float) );
	v_prev		= (float *) malloc ( size*sizeof(float) );
	dens		= (float *) malloc ( size*sizeof(float) );
	dens_prev	= (float *) malloc ( size*sizeof(float) );
	object_space_grid = (int *) malloc(size*sizeof(int));
	objectRed = (int *) malloc(redSize*sizeof(int));
	uRed	= (float *) malloc ( redSize*sizeof(float) );
	vRed	= (float *) malloc ( redSize*sizeof(float) );
	densRed	= (float *) malloc ( redSize*sizeof(float) );
	SAT = (float *) malloc(size*sizeof(float));

	if (!u || !v || !u_prev || !v_prev || !dens || !dens_prev || !object_space_grid ||
			!objectRed || !uRed || !vRed || ! densRed || !SAT)
	{
		fprintf ( stderr, "cannot allocate data\n" );
		return ( 0 );
	}

	return ( 1 );
}

/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	glutSwapBuffers ();
}

static void draw_velocity ( void )
{
	int i, j;
	float x, y, h;
	int NRed = N/sizeDown;

	glColor3f ( 1.0f, 1.0f, 1.0f );
	glLineWidth ( 1.0f );

	glBegin ( GL_LINES );

	if (!lowRes)
	{
		h = 1.0f/N;

		for (i = 1 ; i <= N ; i++)
		{
			x = (i-0.5f)*h;
			for (j = 1 ; j <= N ; j++)
			{
				y = (j-0.5f)*h;

				glVertex2f ( x, y );
				glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
			}
		}
	}

	else
	{
		h = 1.0f/NRed;

		for (i = 1 ; i <= NRed ; i++)
		{
			x = (i-0.5f)*h;
			for (j = 1 ; j <= NRed ; j++)
			{
				y = (j-0.5f)*h;

				glVertex2f ( x, y );
				glVertex2f ( x+uRed[IXRed(i,j)], y+vRed[IXRed(i,j)] );
			}
		}
	}

	glEnd ();
}

static void draw_density ( void )
{
	int i, j;
	float x, y, h, d00, d01, d10, d11;
	int NRed = N/sizeDown;

	glBegin ( GL_QUADS );

	if (!lowRes)
	{
		h = 1.0f/N;
		for ( i=0 ; i<=N ; i++ )
		{
			x = (i-0.5f)*h;
			for ( j=0 ; j<=N ; j++ )
			{
				y = (j-0.5f)*h;

				d00 = dens[IX(i,j)];
				d01 = dens[IX(i,j+1)];
				d10 = dens[IX(i+1,j)];
				d11 = dens[IX(i+1,j+1)];

				glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
				glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
				glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
				glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
			}
		}
	}

	else
	{
		h = 1.0f/NRed;
		for ( i=0 ; i<=NRed ; i++ )
		{
			x = (i-0.5f)*h;
			for ( j=0 ; j<=NRed ; j++ )
			{
				y = (j-0.5f)*h;

				d00 = densRed[IXRed(i,j)];
				d01 = densRed[IXRed(i,j+1)];
				d10 = densRed[IXRed(i+1,j)];
				d11 = densRed[IXRed(i+1,j+1)];

				glColor3f ( d00, d00, d00 ); glVertex2f ( x, y );
				glColor3f ( d10, d10, d10 ); glVertex2f ( x+h, y );
				glColor3f ( d11, d11, d11 ); glVertex2f ( x+h, y+h );
				glColor3f ( d01, d01, d01 ); glVertex2f ( x, y+h );
			}
		}
	}

	glEnd ();
}

void draw_object()
{
	int i, j;
	float x, y, h;
	int NRed = N/sizeDown;



	glBegin ( GL_QUADS );

	if (!lowRes)
	{
		h = 1.0f/N;


		for ( i=0 ; i<=N ; i++ )
		{
			x = (i-0.5f)*h;
			for ( j=0 ; j<=N ; j++ )
			{
				y = (j-0.5f)*h;

				if (object_space_grid[IX(i, j)] == 1)
				{
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x, y);
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x + h, y);
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x + h, y + h);
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x, y + h);
				}
			}
		}
	}

	else
	{
		h = 1.0f/NRed;


		for ( i=0 ; i<=NRed ; i++ )
		{
			x = (i-0.5f)*h;
			for ( j=0 ; j<=NRed ; j++ )
			{
				y = (j-0.5f)*h;

				if (objectRed[IXRed(i, j)] == 1)
				{
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x, y);
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x + h, y);
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x + h, y + h);
					glColor3f(0.5, 0.0, 1.0);
					glVertex2f(x, y + h);
				}
			}
		}
	}

	glEnd ();
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

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI ( float * d, float * u, float * v )
{
	int i, j, size = (N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = d[i] = 0.0f;
	}

	if ( !mouse_down[0] && !mouse_down[2] ) return;

	i = (int)((mx/(float)win_x)*N+1);
	j = (int)(((win_y-my)/(float)win_y)*N+1);

	if ( i<1 || i>N || j<1 || j>N ) return;

//	if ( mouse_down[0] ) {
//		u[IX(i,j)] = force * (mx-omx);
//		v[IX(i,j)] = force * (omy-my);
//	}

	if ( mouse_down[2] ) {
		d[IX(i,j)] = source;
	}

	omx = mx;
	omy = my;

	return;
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 'c':
		case 'C':
			clear_data ();
			break;

		case 'q':
		case 'Q':
			free_data ();
			exit ( 0 );
			break;

		case 'v':
		case 'V':
			dvel = !dvel;
			break;

		case 'o':
		case 'O':
			objects = !objects;
			break;

		case 'l':
		case 'L':
			lowRes = !lowRes;
			break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
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

	get_from_UI ( dens_prev, u_prev, v_prev );
	vel_step ( N, u, v, u_prev, v_prev, visc, dt, objects, object_space_grid );
	dens_step ( N, dens, dens_prev, u, v, diff, dt, objects, object_space_grid );

	if (lowRes)
	{
		reduce(u, uRed, 1);
		reduce(v, vRed, 2);
		reduce(dens, densRed, 0);
	}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	if (dvel)
	{
		draw_velocity ();
	}
	else
	{
		draw_density ();
	}

	if (objects)
	{
		draw_object();
	}

	post_display ();
}

/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "CFD" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}

/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc != 1 && argc != 7 && argc != 8 && argc != 9 && argc != 10) {
		fprintf ( stderr, "usage : %s N dt diff visc force source sources objects sizeDown\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
		fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
		fprintf ( stderr, "\t objects : file with data about objects in the map\n" );
		fprintf ( stderr, "\t sources : file with data about sources in the map\n" );
		fprintf ( stderr, "\t sizeDown : numerical factor to reduce resolution, or the square submatrix size\n" );
		exit ( 1 );
	}

	if ( argc == 1 )
	{
		N = 64;
		dt = 0.1f;
		diff = 0.0f;
		visc = 0.0f;
		force = 5.0f;
		source = 100.0f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
			N, dt, diff, visc, force, source );
	}

	else
	{
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		diff = atof(argv[3]);
		visc = atof(argv[4]);
		force = atof(argv[5]);
		source = atof(argv[6]);

		if (argc == 8)
		{
			o_name = argv[7];
		}

		else if (argc == 9)
		{
			o_name = argv[7];
			s_name = argv[8];
			sizeDown = 0;
		}

		else
		{
			o_name = argv[7];
			s_name = argv[8];
			sizeDown = atoi(argv[9]);
		}
	}

	printf ( "\n\nHow to use this demo:\n\n" );
	printf ( "\t Add densities with the right mouse button\n" );
	printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
	printf ( "\t Toggle density/velocity display with the 'v' key\n" );
	printf ( "\t Clear the simulation by pressing the 'c' key\n" );
	printf ( "\t Add the objects (if specified) by pressing the 'o' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dvel = 0;
	objects = 0;
	lowRes = 0;

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();

	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}



