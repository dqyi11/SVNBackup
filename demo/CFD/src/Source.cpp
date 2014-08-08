/*
 * Source.cpp
 *
 *  Created on: Jun 25, 2014
 *      Author: dhruvmsaxena
 */

#include "Source.h"

Source::Source()
{
	type = -1;
	o_x = -1;
	o_y = -1;
	param1 = -1;
	param2 = -1;
	strength = -1;
	slope = -1;
}

Source::Source(float params[7])
{
	type = int(params[0]);
	o_x = int(params[1]);
	o_y = int(params[2]);
	param1 = params[3];
	param2 = params[4];
	strength = params[5];
	slope = params[6];
}

void Source::setSource(float *v_x, float *v_y, int width, int height)
{
	if (type == 0)
	{
		float u, v;
		u = strength * cos(param2);
		v = strength * sin(param2);

		for (float dist = 1.0; dist <= param1/2; dist += 0.1)
		{
			int a, b;
			a = dist*cos(slope);
			b = dist*sin(slope);

			v_x[((o_x + a)+(width+2)*(height-o_y + b))] = u;
			v_y[((o_x + a)+(width+2)*(height-o_y + b))] = v;
			v_x[((o_x - a)+(width+2)*(height-o_y - b))] = u;
			v_y[((o_x - a)+(width+2)*(height-o_y - b))] = v;
		}
	}

	else if (type == 1)
	{
		int x, y;

		for (float a = param1; a <= param2; a += 0.1)
		{
			x = o_x + (strength * cos(a));
			y = o_y + (strength * sin(a));

			float dist = sqrt((x - o_x)*(x - o_x) + (y - o_y)*(y - o_y));
			float angle = atan2(y - o_y, x - o_x);

			v_x[((x)+(width+2)*(height-y))] = (dist * cos(angle))/strength;
			v_y[((x)+(width+2)*(height-y))] = (dist * sin(angle))/strength;

			//		for (float p = 0.0; p <= 3.0; p += 0.3)
			//		{
			//			x = N/2 + (3.0 * cos(a));
			//			y = N/2 + (3.0 * sin(a));
			//
			//			float dist = sqrt((x - N/2)*(x - N/2) + (y - N/2)*(y - N/2));
			//			float angle = atan2(y-N/2, x-N/2);
			//
			//			if (p)
			//			{
			//				u[IX(x, y)] = (dist * cos(angle))/3.0;
			//				v[IX(x, y)] = (dist * sin(angle))/3.0;
			//			}
			//		}
		}
	}
}

Source::~Source() {
	// TODO Auto-generated destructor stub
}

