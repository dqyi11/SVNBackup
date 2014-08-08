/*
 * solver.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: dhruvmsaxena
 */

#define IX(i,j) ((W+2)*(j)+(i))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=W ; i++ ) { for ( j=1 ; j<=H ; j++ ) {
#define END_FOR }}

void add_source ( int W, int H, float * x, float * s, float dt )
{
	int i, size=(W+2)*(H+2);
	for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void set_bnd ( int W, int H, int b, float * x, int o, int * object )
{
	int i,j;

	if(o)
	{
		for(i=1; i<=W; i++)
		{
			for(j=1; j<=H; j++)
			{
				if(object[IX(i,j)])
				{
					x[IX(i-1, j)] = (b==1 ? -x[IX(i-2,j)] : x[IX(i-2,j)]);
					x[IX(i+1, j)] = (b==1 ? -x[IX(i+2,j)] : x[IX(i+2,j)]);
					x[IX(i, j-1)] = (b==2 ? -x[IX(i,j-2)] : x[IX(i,j-2)]);
					x[IX(i, j+1)] = (b==2 ? -x[IX(i,j+2)] : x[IX(i,j+2)]);
				}
			}
		}
	}

	for ( j=1 ; j<=W ; j++ )
	{
		x[IX(0  ,j)] = b==1 ? -x[IX(1,j)] : x[IX(1,j)];
		x[IX(H+1,j)] = b==1 ? x[IX(H,j)] : x[IX(H,j)];
		
	}
	for ( i=1 ; i<=H ; i++ )
	{
		x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
		x[IX(i,W+1)] = b==2 ? -x[IX(i,W)] : x[IX(i,W)];
	}
	x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
	x[IX(0  ,W+1)] = 0.5f*(x[IX(1,W+1)]+x[IX(0  ,W)]);
	x[IX(H+1,0  )] = 0.5f*(x[IX(H,0  )]+x[IX(H+1,1)]);
	x[IX(H+1,W+1)] = 0.5f*(x[IX(H,W+1)]+x[IX(H+1,W)]);
}

void lin_solve ( int W, int H, int b, float * x, float * x0, float a, float c, int o, int * object )
{
	int i, j, k;

	for ( k=0 ; k<20 ; k++ ) {
		FOR_EACH_CELL
			x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
		END_FOR
		set_bnd ( W, H, b, x, o, object );
	}
}

void diffuse ( int W, int H, int b, float * x, float * x0, float diff, float dt, int o, int * object )
{
	float a=dt*diff*W*H;
	lin_solve ( W, H, b, x, x0, a, 1+4*a, o, object );
}

void advect ( int W, int H, int b, float * d, float * d0, float * u, float * v, float dt, int o, int * object )
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;

	dt0 = dt*((W+H)/2);
	FOR_EACH_CELL
		x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
		if (x<0.5f) x=0.5f; if (x>W+0.5f) x=W+0.5f; i0=(int)x; i1=i0+1;
		if (y<0.5f) y=0.5f; if (y>H+0.5f) y=H+0.5f; j0=(int)y; j1=j0+1;
		s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
		d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
					 s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
	END_FOR
	set_bnd ( W, H, b, d, o, object );
}

void project ( int W, int H, float * u, float * v, float * p, float * div, int o, int * object )
{
	int i, j;

	FOR_EACH_CELL
		div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/((W+H)/2);
		p[IX(i,j)] = 0;
	END_FOR
	set_bnd ( W, H, 0, div, o, object ); set_bnd ( W, H, 0, p, o, object );

	lin_solve ( W, H, 0, p, div, 1, 4, o, object );

	FOR_EACH_CELL
		u[IX(i,j)] -= 0.5f*((W+H)/2)*(p[IX(i+1,j)]-p[IX(i-1,j)]);
		v[IX(i,j)] -= 0.5f*((W+H)/2)*(p[IX(i,j+1)]-p[IX(i,j-1)]);
	END_FOR
	set_bnd ( W, H, 1, u, o, object ); set_bnd ( W, H, 2, v, o, object );
}

void vel_step ( int W, int H, float * u, float * v, float * u0, float * v0, float visc, float dt, int o, int * object )
{
	add_source ( W, H, u, u0, dt );
	add_source ( W, H, v, v0, dt );
//	SWAP ( u0, u );
	diffuse ( W, H, 1, u0, u, visc, dt, o, object );
//	SWAP ( v0, v );
	diffuse ( W, H, 2, v0, v, visc, dt, o, object );
	project ( W, H, u0, v0, u, v, o, object );
//	SWAP ( u0, u );
//	SWAP ( v0, v );
	advect ( W, H, 1, u, u0, u0, v0, dt, o, object );
	advect ( W, H, 2, v, v0, u0, v0, dt, o, object );
	project ( W, H, u, v, u0, v0, o, object );
}




