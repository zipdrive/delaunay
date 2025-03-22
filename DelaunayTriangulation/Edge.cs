using System;
using System.Numerics;

namespace DelaunayTriangulation;

/// <summary>
/// An edge between two vertices.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Vertex">The data type for each vertex.</typeparam>
public class Edge<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
    internal Vertex Vertex1;
    internal Vertex Vertex2;

	/// <summary>
	/// The vertices of this edge.
	/// </summary>
	public IEnumerable<Vertex> Vertices
	{
		get
		{
			yield return Vertex1;
			yield return Vertex2;
		}
	}

	/// <summary>
	/// The vector parallel to the edge.
	/// </summary>
	internal Vector2<T> Vector => Vector2<T>.VectorDifference(Vertex1, Vertex2);

    /// <summary>
    /// The vector perpendicular to the edge.
    /// </summary>
    internal Vector2<T> Normal => new Vector2<T>
    {
        X = Vertex2.Y - Vertex1.Y,
        Y = Vertex1.X - Vertex2.X
    };

	/// <summary>
	/// The triangle on the left side of this edge.
	/// </summary>
	internal Triangle<T, Vertex>? _Left;

	/// <summary>
	/// The triangle on the left side of this edge.
	/// </summary>
	public Triangle<T, Vertex>? Left => _Left;

	/// <summary>
	/// The triangle on the right side of this edge.
	/// </summary>
	internal Triangle<T, Vertex>? _Right;

	/// <summary>
	/// The triangle on the right side of this edge.
	/// </summary>
	public Triangle<T, Vertex>? Right => _Right;

	/// <summary>
	/// Creates an edge between the two given vertices.
	/// </summary>
	/// <param name="vertex1">An endpoint vertex of the edge.</param>
	/// <param name="vertex2">An endpoint vertex of the edge.</param>
	internal Edge(Vertex vertex1, Vertex vertex2)
	{
		Vertex1 = vertex1;
		Vertex2 = vertex2;
	}

	/// <summary>
	/// Calculates the offset of a vertex from the right side of this edge.
	/// </summary>
	/// <param name="vertex">The vertex to find the offset of.</param>
	/// <returns>The right-hand offset of the vertex from the edge.</returns>
	internal T GetRighthandOffset(Vertex vertex)
	{
		return Normal.Dot(new Vector2<T>
		{
			X = vertex.X - Vertex1.X,
			Y = vertex.Y - Vertex1.Y
		});
	}

	/// <summary>
	/// Calculates the angle between two edges, connected at one vertex.
	/// </summary>
	/// <param name="other">The other edge.</param>
	/// <returns>The counter-clockwise angle between the two vertices.</returns>
	/// <exception cref="Exception">Thrown if the given edge is disconnected from this one.</exception>
	public T GetAngularDifference(Edge<T, Vertex> other)
    {
        Vector2<T> b1Vector = Vector;
        Vector2<T> b2Vector = other.Vector;

        if (other.Vertex1.Equals(Vertex2))
		{
			b2Vector = -b2Vector;
		}
		else if (other.Vertex2.Equals(Vertex1))
		{
			b1Vector = -b1Vector;
		}
		else if (other.Vertex2.Equals(Vertex2))
		{
			b1Vector = -b1Vector;
			b2Vector = -b2Vector;
		}
		else if (!other.Vertex1.Equals(Vertex1))
		{
			throw new ArgumentException("The edges are disconnected, so the angle between them cannot be calculated.");
		}

		return b1Vector.Angle(b2Vector);
	}

	/// <summary>
	/// Flips the order of vertices in the edge.
	/// </summary>
	internal void Flip()
	{
		Vertex vtemp = Vertex1;
		Vertex1 = Vertex2;
		Vertex2 = vtemp;

		Triangle<T, Vertex>? ttemp = Left;
		_Left = Right;
		_Right = ttemp;
	}


    public override bool Equals(object? obj)
    {
		if (obj is Edge<T, Vertex> other)
			return other.Vertex1.Equals(Vertex1) && other.Vertex2.Equals(Vertex2);
		return false;
    }

    public override int GetHashCode()
    {
		bool ordering;
		int compareX = Vertex1.X.CompareTo(Vertex2.X);
		if (compareX != 0)
			ordering = compareX < 0;
		else
			ordering = Vertex1.Y.CompareTo(Vertex2.Y) < 0;
		return ordering ? HashCode.Combine(Vertex1, Vertex2) : HashCode.Combine(Vertex2, Vertex1);
    }
}
