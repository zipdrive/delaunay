using Retriangulator2D;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulationTestCase1;

public class Vertex2 : IPoint2<double>
{
	public double X { get; set; }

	public double Y { get; set; }

	public override string ToString()
	{
		return $"({X:F2}, {Y:F2})";
	}
}
