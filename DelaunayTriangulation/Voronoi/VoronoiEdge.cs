using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulation.Voronoi;

/// <summary>
/// An edge of a region in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class VoronoiEdge<T> where T : IFloatingPointIeee754<T>
{

}

/// <summary>
/// An edge of finite length in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class VoronoiBoundedEdge<T> : VoronoiEdge<T> where T : IFloatingPointIeee754<T>
{

}

/// <summary>
/// An edge of infinite length in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class VoronoiUnboundedEdge<T> : VoronoiEdge<T> where T : IFloatingPointIeee754<T>
{
	
}

