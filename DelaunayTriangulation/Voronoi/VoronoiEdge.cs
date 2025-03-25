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
public interface VoronoiEdge<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	/// <summary>
	/// The vertices that define the edge.
	/// </summary>
	IEnumerable<IVertex2<T>> Vertices { get; }

	/// <summary>
	/// The region on the left side of the edge.
	/// </summary>
	VoronoiRegion<T, Vertex> Left { get; }

	/// <summary>
	/// The region on the right side of the edge.
	/// </summary>
	VoronoiRegion<T, Vertex> Right { get; }
}

/// <summary>
/// An edge of finite length in a Voronoi diagram.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class VoronoiBoundedEdge<T, Vertex> : VoronoiEdge<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
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

	public VoronoiRegion<T, Vertex> Left { get; private set; }
	public VoronoiRegion<T, Vertex> Right { get; private set; }

	internal VoronoiBoundedEdge(IVertex2<T> vertex1, IVertex2<T> vertex2, VoronoiRegion<T, Vertex> left, VoronoiRegion<T, Vertex> right)
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
public class VoronoiUnboundedEdge<T, Vertex> : VoronoiEdge<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	internal readonly IVertex2<T> Vertex1;

	public IEnumerable<IVertex2<T>> Vertices
	{
		get
		{
			yield return Vertex1;
		}
	}

	public VoronoiRegion<T, Vertex> Left { get; private set; }
	public VoronoiRegion<T, Vertex> Right { get; private set; }

	internal VoronoiUnboundedEdge(IVertex2<T> vertex1, VoronoiRegion<T, Vertex> left, VoronoiRegion<T, Vertex> right)
	{
		Vertex1 = vertex1;
		Left = left;
		Right = right;
	}
}

