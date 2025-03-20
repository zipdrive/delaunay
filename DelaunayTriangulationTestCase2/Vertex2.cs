using DelaunayTriangulation;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulationTestCase2
{
	public class Vertex2 : IVertex2<double>
	{
		public double X { get; set; }

		public double Y { get; set; }
	}
}
