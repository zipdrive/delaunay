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
