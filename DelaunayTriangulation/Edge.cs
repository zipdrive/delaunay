using System;
using System.Numerics;

namespace DelaunayTriangulation;

internal class Edge<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
    public required Vertex Vertex1;
    public required Vertex Vertex2;

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
	public Triangle<T, Vertex>? Left;

	/// <summary>
	/// The triangle on the right side of this edge.
	/// </summary>
	public Triangle<T, Vertex>? Right;

	/// <summary>
	/// Calculates the offset of a vertex from the right side of this edge.
	/// </summary>
	/// <param name="vertex">The vertex to find the offset of.</param>
	/// <returns>The right-hand offset of the vertex from the edge.</returns>
	public T GetRighthandOffset(Vertex vertex)
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
		Vertex a, b1, b2;
		if (other.Vertex1.Equals(Vertex1))
		{
			a = Vertex1;
			b1 = Vertex2;
			b2 = other.Vertex2;
		}
		else if (other.Vertex1.Equals(Vertex2))
		{
			a = Vertex2;
			b1 = Vertex1;
			b2 = other.Vertex2;
		}
		else if (other.Vertex2.Equals(Vertex1))
		{
			a = Vertex1;
			b1 = Vertex2;
			b2 = other.Vertex1;
		}
		else if (other.Vertex2.Equals(Vertex2))
		{
			a = Vertex2;
			b1 = Vertex1;
			b2 = other.Vertex1;
		}
		else
		{
			throw new ArgumentException("The edges are disconnected, so the angle between them cannot be calculated.");
		}

		Vector2<T> b1Vector = Vector;
		if (b1.Equals(Vertex1))
			b1Vector = -b1Vector;
		Vector2<T> b2Vector = other.Vector;
		if (b2.Equals(other.Vertex1))
			b2Vector = -b2Vector;

		T angle = T.Acos(b1Vector.Dot(b2Vector) / (b1Vector.Length + b2Vector.Length));
		T sgn = b1Vector.X * b2Vector.Y - b1Vector.Y * b2Vector.X;
		return sgn < Mesh<T, Vertex>.NumericTolerance ? T.Tau - angle : angle;
	}

	/// <summary>
	/// Flips the order of vertices in the edge.
	/// </summary>
	public void Flip()
	{
		Vertex vtemp = Vertex1;
		Vertex1 = Vertex2;
		Vertex2 = vtemp;

		Triangle<T, Vertex>? ttemp = Left;
		Left = Right;
		Right = ttemp;
	}
}
