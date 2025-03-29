using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Retriangulator2D.Voronoi;

/// <summary>
/// An edge of a region in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Centroid">The data type for each region's centroid.</typeparam>
public interface VoronoiEdge<T, Centroid> where T : IFloatingPointIeee754<T> where Centroid : IVertex2<T>
{
	/// <summary>
	/// The vertices that define the edge.
	/// </summary>
	IEnumerable<IVertex2<T>> Vertices { get; }

	/// <summary>
	/// The region on the left side of the edge.
	/// </summary>
	VoronoiRegion<T, Centroid> Left { get; }

	/// <summary>
	/// The region on the right side of the edge.
	/// </summary>
	VoronoiRegion<T, Centroid> Right { get; }
}

/// <summary>
/// An edge of finite length in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class VoronoiBoundedEdge<T, Centroid> : VoronoiEdge<T, Centroid> where T : IFloatingPointIeee754<T> where Centroid : IVertex2<T>
{
	internal readonly IVertex2<T> Vertex1;
	internal readonly IVertex2<T> Vertex2;

	public IEnumerable<IVertex2<T>> Vertices
	{
		get
		{
			yield return Vertex1;
			yield return Vertex2;
		}
	}

	public VoronoiRegion<T, Centroid> Left { get; private set; }
	public VoronoiRegion<T, Centroid> Right { get; private set; }

	internal VoronoiBoundedEdge(IVertex2<T> vertex1, IVertex2<T> vertex2, VoronoiRegion<T, Centroid> left, VoronoiRegion<T, Centroid> right)
	{
		Vertex1 = vertex1;
		Vertex2 = vertex2;
		Left = left;
		Right = right;
	}
}

/// <summary>
/// An edge of infinite length in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class VoronoiUnboundedEdge<T, Centroid> : VoronoiEdge<T, Centroid> where T : IFloatingPointIeee754<T> where Centroid : IVertex2<T>
{
	internal readonly IVertex2<T> Vertex1;
	internal readonly Vector2<T> EdgeVector;

	public IEnumerable<IVertex2<T>> Vertices
	{
		get
		{
			yield return Vertex1;
		}
	}

	public VoronoiRegion<T, Centroid> Left { get; private set; }
	public VoronoiRegion<T, Centroid> Right { get; private set; }

	internal VoronoiUnboundedEdge(IVertex2<T> vertex1, Vector2<T> edgeVector, VoronoiRegion<T, Centroid> left, VoronoiRegion<T, Centroid> right)
	{
		Vertex1 = vertex1;
		EdgeVector = edgeVector;
		Left = left;
		Right = right;
	}
}

