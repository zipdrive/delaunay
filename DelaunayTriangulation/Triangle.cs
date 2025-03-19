using System;
using System.Numerics;

namespace DelaunayTriangulation;

public class Triangle<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	internal Edge<T, Vertex> Edge1;
	internal Edge<T, Vertex> Edge2;
	internal Edge<T, Vertex> Edge3;

	/// <summary>
	/// The x-coordinate of the center of the circumcircle defined by these three vertices.
	/// </summary>
	public T CircumcircleCenterX;

	/// <summary>
	/// The y-coordiante of the center of the circumcircle defined by these three vertices.
	/// </summary>
	public T CircumcircleCenterY;

	/// <summary>
	/// The radius of the circumcircle defined by these three vertices, squared.
	/// </summary>
	private T CircumcircleRadiusSquared;

	/// <summary>
	/// The vertices of this triangle.
	/// </summary>
	public IEnumerable<Vertex> Vertices => 
		new Vertex[] { Edge1.Vertex1, Edge1.Vertex2, Edge2.Vertex1, Edge2.Vertex2, Edge3.Vertex1, Edge3.Vertex2 }
		.Distinct();

	/// <summary>
	/// The edges of this triangle.
	/// </summary>
	internal IEnumerable<Edge<T, Vertex>> Edges
	{
		get
		{
			yield return Edge1;
			yield return Edge2;
			yield return Edge3;
		}
	}

	/// <summary>
	/// The triangles adjacent to this one.
	/// </summary>
	public IEnumerable<Triangle<T, Vertex>> AdjacentTriangles => 
		new Triangle<T, Vertex>?[] { Edge1.Left, Edge1.Right, Edge2.Left, Edge2.Right, Edge3.Left, Edge3.Right }
		.Where(triangle => triangle != null && !triangle.Equals(this))
		.Cast<Triangle<T, Vertex>>();

	/// <summary>
	/// Constructs a triangle from three vertices, for a mesh that consists of only three vertices.
	/// Flips the ordering of existing edges to ensure that the edges are in a counter-clockwise direction, then updates the edge adjacencies.
	/// </summary>
	/// <param name="mesh">The existing triangulation.</param>
	/// <param name="vertex1">A vertex of the triangle.</param>
	/// <param name="vertex2">A vertex of the triangle.</param>
	/// <param name="vertex3">A vertex of the triangle.</param>
	internal Triangle(Mesh<T, Vertex> mesh, Vertex vertex1, Vertex vertex2, Vertex vertex3)
	{
		Edge1 = mesh.FindOrCreateEdge(vertex1, vertex2);
		if (Edge1.GetRighthandOffset(vertex3) > Mesh<T, Vertex>.NumericTolerance)
			Edge1.Flip();
		Edge1.Left = this;

		Edge2 = mesh.FindOrCreateEdge(Edge1.Vertex2, vertex3);
		if (!Edge2.Vertex1.Equals(Edge1.Vertex2))
			Edge2.Flip();
		Edge2.Left = this;

		Edge3 = mesh.FindOrCreateEdge(vertex3, Edge1.Vertex1);
		if (!Edge3.Vertex1.Equals(Edge2.Vertex2))
			Edge3.Flip();
		Edge3.Left = this;

		RecalculateCircumcircle(vertex1, vertex2, vertex3);
	}

	/// <summary>
	/// Constructs a triangle from an edge and a third vertex.
	/// </summary>
	/// <param name="mesh">The existing triangulation.</param>
	/// <param name="edge">The existing edge.</param>
	/// <param name="vertex3">The third vertex of the triangle.</param>
	internal Triangle(Mesh<T, Vertex> mesh, Edge<T, Vertex> edge, Vertex vertex3)
	{
		Edge1 = edge;
		Edge2 = mesh.FindOrCreateEdge(edge.Vertex2, vertex3);
		Edge3 = mesh.FindOrCreateEdge(vertex3, edge.Vertex1);

		RecalculateCircumcircle(edge.Vertex1, edge.Vertex2, vertex3);
	}

	/// <summary>
	/// Calculates the circumcircle center and radius.
	/// </summary>
	public void RecalculateCircumcircle(Vertex vertex1, Vertex vertex2, Vertex vertex3)
	{
		// Calculate circumcircle
		T two = T.CreateChecked(2);
		Vector2<T> offset12 = Vector2<T>.VectorDifference(vertex1, vertex2).Normalize();
		Vector2<T> offset23 = Vector2<T>.VectorDifference(vertex2, vertex3).Normalize();
		Vector2<T> offset31 = Vector2<T>.VectorDifference(vertex3, vertex1).Normalize();
		T angle1 = T.Acos(-offset12.Dot(offset31));
		T sin1 = T.Sin(two * angle1);
		T angle2 = T.Acos(-offset23.Dot(offset12));
		T sin2 = T.Sin(two * angle2);
		T angle3 = T.Acos(-offset31.Dot(offset23));
		T sin3 = T.Sin(two * angle3);
		T normalizationConstant = T.One / (sin1 + sin2 + sin3);
		CircumcircleCenterX = normalizationConstant * (vertex1.X * sin1 + vertex2.X * sin2 + vertex3.X * sin3);
		CircumcircleCenterY = normalizationConstant * (vertex1.Y * sin1 + vertex2.Y * sin2 + vertex3.Y * sin3);
		CircumcircleRadiusSquared = T.Pow(CircumcircleCenterX - vertex3.X, two) + T.Pow(CircumcircleCenterY - vertex3.Y, two);
	}

	/// <summary>
	/// Checks if the point exists inside the circumcircle defined by the three vertices of this triangle.
	/// </summary>
	/// <param name="vertex">A point to check.</param>
	/// <returns>True if the point exists inside the circumcircle. False if the point is on the boundary of the circumcircle or fully outside of the circumcircle.</returns>
	public bool IsInsideCircumcircle(Vertex vertex) => new Vector2<T>
	{
		X = vertex.X - CircumcircleCenterX,
		Y = vertex.Y - CircumcircleCenterY
	}.LengthSquared < CircumcircleRadiusSquared + Mesh<T, Vertex>.NumericTolerance;

	/// <summary>
	/// Updates the edges composing this triangle to point back to this triangle.
	/// </summary>
	/// <exception cref="Exception">Throws an exception if the three vertices of the triangle are on a single line.</exception>
	internal void UpdateAdjacentEdges()
	{
		HashSet<Vertex> vertices = new HashSet<Vertex>(Vertices);
		foreach (Edge<T, Vertex> edge in Edges)
		{
			Vertex otherVertex = vertices.First(v => !v.Equals(edge.Vertex1) && !v.Equals(edge.Vertex2));
			T righthandOffset = edge.GetRighthandOffset(otherVertex);
			if (righthandOffset > Mesh<T, Vertex>.NumericTolerance)
				edge.Right = this;
			else if (righthandOffset < -Mesh<T, Vertex>.NumericTolerance)
				edge.Left = this;
			else
			{
				// Degenerate case
				throw new Exception("One of the triangles constructed had three vertices on a single line.");
			}
		}
	}

    public override bool Equals(object? obj)
    {
        if (obj is Triangle<T, Vertex> other)
            return !Edges.Except(other.Edges).Any();
        return false;
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Edge1, Edge2, Edge3);
    }
}
