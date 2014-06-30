using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace PathPlanner.Hexagonal
{
	public class HexState
	{
		private System.Drawing.Color backgroundColor;
		

		public System.Drawing.Color BackgroundColor
		{
			get
			{
				return backgroundColor;
			}
			set
			{
				backgroundColor = value;
			}
		}


		public HexState()
		{
			this.backgroundColor = Color.White;
		}

	}
}
