using System;
using System.Numerics;

namespace Retriangulator2D;

/// <summary>
/// A triangle of three vertices.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Vertex">The data type for each vertex.</typeparam>
public class Triangle<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	internal readonly Edge<T, Vertex> Edge1;
	internal readonly Edge<T, Vertex> Edge2;
	internal readonly Edge<T, Vertex> Edge3;

	/// <summary>
	/// The center of the circumcircle defined by these three vertices.
	/// </summary>
	public readonly IVertex2<T> CircumcircleCenter;

	/// <summary>
	/// The radius of the circumcircle defined by these three vertices, squared.
	/// </summary>
	public readonly T CircumcircleRadiusSquared;

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
	public IEnumerable<Triangle<T, Vertex>> AdjacentTriangles
	{
		get
		{
			if (Edge1.Left != null && Edge1.Left != this)
				yield return Edge1.Left;
			if (Edge1.Right != null && Edge1.Right != this)
				yield return Edge1.Right;
			if (Edge2.Left != null && Edge2.Left != this)
				yield return Edge2.Left;
			if (Edge2.Right != null && Edge2.Right != this)
				yield return Edge2.Right;
			if (Edge3.Left != null && Edge3.Left != this)
				yield return Edge3.Left;
			if (Edge3.Right != null && Edge3.Right != this)
				yield return Edge3.Right;
		}
	}

	/// <summary>
	/// The area of the triangle.
	/// </summary>
	public T Area
	{
		get
		{
			Vector2<T> vector1 = Edge1.Vector;
			Vector2<T> vector2 = Edge2.Vector;

			if (Edge2.Vertex1.Equals(Edge1.Vertex2))
			{
				vector2 = -vector2;
			}
			else if (Edge2.Vertex2.Equals(Edge1.Vertex1))
			{
				vector1 = -vector1;
			}

			return T.CreateChecked(0.5) * T.Abs(vector1.X * vector2.Y - vector1.Y * vector2.X);
		}
	}

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
		if (Edge1.GetRighthandOffset(vertex3) > mesh.NumericTolerance)
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

		(CircumcircleCenter, CircumcircleRadiusSquared) = _RecalculateCircumcircle(vertex1, vertex2, vertex3);
	}

	/// <summary>
	/// Constructs a triangle from a list of three edges.
	/// </summary>
	/// <param name="edges"></param>
	internal Triangle(List<Edge<T, Vertex>> edges)
	{
		if (edges.Count < 3)
			throw new ArgumentException("Must provide three connected edges to constructor for triangle.");
		Edge1 = edges[0];
		Edge2 = edges[1];
		Edge3 = edges[2];

		(CircumcircleCenter, CircumcircleRadiusSquared) = _RecalculateCircumcircle(Edge1.Vertex1, Edge1.Vertex2, Edge2.Vertices.Except(Edge1.Vertices).First());
	}

	/// <summary>
	/// Constructs a triangle from two connected edges.
	/// </summary>
	/// <param name="mesh">The existing triangulation.</param>
	/// <param name="edge1">An existing edge.</param>
	/// <param name="edge2">An existing edge.</param>
	internal Triangle(Mesh<T, Vertex> mesh, Edge<T, Vertex> edge1, Edge<T, Vertex> edge2)
	{
		Edge1 = edge1;
		Edge2 = edge2;
		Vertex[] vertex = new Vertex[3];
		if (edge1.Vertex1.Equals(edge2.Vertex1))
		{
			vertex[0] = edge1.Vertex2;
			vertex[1] = edge1.Vertex1;
			vertex[2] = edge2.Vertex2;
		}
		else if (edge1.Vertex1.Equals(edge2.Vertex2))
		{
			vertex[0] = edge1.Vertex2;
			vertex[1] = edge1.Vertex1;
			vertex[2] = edge2.Vertex1;
		}
		else if (edge1.Vertex2.Equals(edge2.Vertex1))
		{
			vertex[0] = edge1.Vertex1;
			vertex[1] = edge1.Vertex2;
			vertex[2] = edge2.Vertex2;
		}
		else if (edge1.Vertex2.Equals(edge2.Vertex2))
		{
			vertex[0] = edge1.Vertex1;
			vertex[1] = edge1.Vertex2;
			vertex[2] = edge2.Vertex1;
		}
		else
			throw new ArgumentException("Must provide two connected edges to constructor for triangle.");
		Edge3 = mesh.FindOrCreateEdge(vertex[2], vertex[0]);

		(CircumcircleCenter, CircumcircleRadiusSquared) = _RecalculateCircumcircle(vertex[0], vertex[1], vertex[2]);
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

		(CircumcircleCenter, CircumcircleRadiusSquared) = _RecalculateCircumcircle(edge.Vertex1, edge.Vertex2, vertex3);
	}

	/// <summary>
	/// Calculates the circumcircle center and radius.
	/// </summary>
	private (SimpleVertex2<T>, T) _RecalculateCircumcircle(Vertex vertex1, Vertex vertex2, Vertex vertex3)
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
		SimpleVertex2<T> center = new SimpleVertex2<T>
		{
			X = normalizationConstant * (vertex1.X * sin1 + vertex2.X * sin2 + vertex3.X * sin3),
			Y = normalizationConstant * (vertex1.Y * sin1 + vertex2.Y * sin2 + vertex3.Y * sin3)
		};
		T radiusSquared = T.Pow(center.X - vertex3.X, two) + T.Pow(center.Y - vertex3.Y, two);
		return (center, radiusSquared);
	}

	/// <summary>
	/// Checks if the point exists inside the circumcircle defined by the three vertices of this triangle.
	/// </summary>
	/// <param name="vertex">A point to check.</param>
	/// <returns>True if the point exists inside the circumcircle. False if the point is on the boundary of the circumcircle or fully outside of the circumcircle.</returns>
	internal bool IsInsideCircumcircle(Vertex vertex) => Vector2<T>.VectorDifference(vertex, CircumcircleCenter).LengthSquared < CircumcircleRadiusSquared;

	/// <summary>
	/// Determines the orientation of the triangle.
	/// </summary>
	/// <returns>Greater than 0 if the triangle is oriented counter-clockwise.
	/// Equal to 0 if the vertices of the triangle are co-linear.
	/// Less than 0 if the triangle is oriented clockwise.</returns>
	internal T Orientation
	{
		get
		{
			Vector2<T> vector1 = Edge1.Vector;
			Vector2<T> vector2 = Edge2.Normal;
			if (Edge1.Vertex1.Equals(Edge2.Vertex1))
				vector1 = -vector1;
			else if (Edge1.Vertex2.Equals(Edge2.Vertex2))
				vector2 = -vector2;
			else if (Edge1.Vertex1.Equals(Edge2.Vertex2))
			{
				vector1 = -vector1;
				vector2 = -vector2;
			}
			return vector1.Dot(vector2);
		}
	}

	/// <summary>
	/// Calculates a power test.
	/// </summary>
	/// <param name="vertex">A point to test.</param>
	/// <returns>The power test of the point.</returns>
	internal T Power(Vertex vertex, T numericTolerance)
	{
		return Orientation > numericTolerance ? 
			CircumcircleRadiusSquared - Vector2<T>.VectorDifference(vertex, CircumcircleCenter).LengthSquared : 
			T.PositiveInfinity;
    }

    /// <summary>
    /// Updates the edges composing this triangle to point back to this triangle.
    /// </summary>
    /// <exception cref="Exception">Throws an exception if the three vertices of the triangle are on a single line.</exception>
    internal void UpdateAdjacentEdges(T numericTolerance)
	{
		HashSet<Vertex> vertices = new HashSet<Vertex>(Vertices);
		foreach (Edge<T, Vertex> edge in Edges)
		{
			Vertex otherVertex = vertices.First(v => !v.Equals(edge.Vertex1) && !v.Equals(edge.Vertex2));
			T righthandOffset = edge.GetRighthandOffset(otherVertex);
			if (righthandOffset > numericTolerance)
				edge.Right = this;
			else if (righthandOffset < -numericTolerance)
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
		{
			return Edges.Intersect(other.Edges).Count() == 3;
		}
		return false;
	}

	public override int GetHashCode()
	{
		return Edge1.GetHashCode() ^ Edge2.GetHashCode() ^ Edge3.GetHashCode();
	}
}
