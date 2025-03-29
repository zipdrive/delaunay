using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Retriangulator2D;

/// <summary>
/// A Delaunay triangulation of vertices.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Vertex">The data type for each vertex.</typeparam>
public abstract class Mesh<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	#region Properties

	/// <summary>
	/// Tolerance for saying a number is approximately equal to another number.
	/// </summary>
	public readonly T NumericTolerance = T.Epsilon;

	#region Delaunay triangulation

	/// <summary>
	/// Maps the vertices of the mesh to the edges of the mesh with that vertex as an endpoint, sorted in counter-clockwise order.
	/// </summary>
	protected Dictionary<Vertex, List<Edge<T, Vertex>>> _Vertices = new Dictionary<Vertex, List<Edge<T, Vertex>>>();

	/// <summary>
	/// The vertices of the mesh.
	/// </summary>
	public IEnumerable<Vertex> Vertices => _Vertices.Keys;

	/// <summary>
	/// The edges of the mesh.
	/// </summary>
	protected HashSet<Edge<T, Vertex>> _Edges = new HashSet<Edge<T, Vertex>>();

	/// <summary>
	/// The edges of the mesh.
	/// </summary>
	public IEnumerable<Edge<T, Vertex>> Edges => _Edges;

    /// <summary>
    /// The edges that form the boundary of the mesh, ordered in a counter-clockwise direction.
    /// </summary>
    public abstract IEnumerable<Edge<T, Vertex>> Boundary { get; }

    /// <summary>
    /// The triangles of the mesh.
    /// </summary>
    protected HashSet<Triangle<T, Vertex>> _Triangles = new HashSet<Triangle<T, Vertex>>();

	/// <summary>
	/// The triangles of the mesh.
	/// </summary>
	public IEnumerable<Triangle<T, Vertex>> Triangles => _Triangles;

	#endregion Delaunay triangulation


	/// <summary>
	/// The total area of the mesh.
	/// </summary>
	public T Area
	{
		get
		{
			if (Triangles.Any())
				return Triangles.Aggregate(T.Zero, (total, triangle) => total + triangle.Area);
			return T.Zero;
		}
	}

	#endregion Properties

	#region Methods

	protected Mesh(T numericTolerance)
	{
		NumericTolerance = numericTolerance;
	}

	/// <summary>
	/// Adds an edge to the mesh.
	/// </summary>
	/// <param name="edge">The edge to insert.</param>
	protected void _AddEdge(Edge<T, Vertex> edge)
	{
		if (!_Edges.Contains(edge))
		{
			_Edges.Add(edge);

			List<Edge<T, Vertex>>? connectedEdges;
			if (_Vertices.TryGetValue(edge.Vertex1, out connectedEdges) && connectedEdges != null)
			{
				if (connectedEdges.Count > 1)
				{
					for (int k = 0; k < connectedEdges.Count; ++k)
					{
						if (connectedEdges[k].GetAngularDifference(edge) < connectedEdges[k].GetAngularDifference(connectedEdges[(k + 1) % connectedEdges.Count]))
						{
							connectedEdges.Insert(k + 1, edge);
							break;
						}
					}
				}
				else connectedEdges.Add(edge);
			}
			else
			{
				_Vertices[edge.Vertex1] = new List<Edge<T, Vertex>> { edge };
			}

			if (_Vertices.TryGetValue(edge.Vertex2, out connectedEdges) && connectedEdges != null)
			{
				if (connectedEdges.Count > 1)
				{
					for (int k = 0; k < connectedEdges.Count; ++k)
					{
						if (connectedEdges[k].GetAngularDifference(edge) < connectedEdges[k].GetAngularDifference(connectedEdges[(k + 1) % connectedEdges.Count]))
						{
							connectedEdges.Insert(k + 1, edge);
							break;
						}
					}
				}
				else connectedEdges.Add(edge);
			}
			else
			{
				_Vertices[edge.Vertex2] = new List<Edge<T, Vertex>> { edge };
			}
		}
	}

	/// <summary>
	/// Adds a triangle to the mesh.
	/// </summary>
	/// <param name="triangle">The triangle to add.</param>
	protected void _AddTriangle(Triangle<T, Vertex> triangle)
	{
		if (!_Triangles.Contains(triangle))
		{
			_Triangles.Add(triangle);
			foreach (Edge<T, Vertex> edge in triangle.Edges)
				_AddEdge(edge);
			triangle.UpdateAdjacentEdges(NumericTolerance);
		}
	}

	/// <summary>
	/// Removes an edge and any triangles associated with that edge, then returns the edge that was removed.
	/// </summary>
	/// <param name="a">A vertex endpoint of the edge.</param>
	/// <param name="b">A vertex endpoint of the edge.</param>
	protected Edge<T, Vertex> _RemoveEdge(Vertex a, Vertex b)
	{
		Edge<T, Vertex>? edge = _Edges.FirstOrDefault(e => (e.Vertex1.Equals(a) && e.Vertex2.Equals(b)) || (e.Vertex2.Equals(a) && e.Vertex1.Equals(b)));
		if (edge == null)
			throw new Exception("Expected to find edge that does not exist.");
		_RemoveEdge(edge);
		return edge;
	}

	/// <summary>
	/// Removes an edge from the mesh, as well as any triangle attached to that edge.
	/// </summary>
	/// <param name="edge">The edge to remove.</param>
	protected void _RemoveEdge(Edge<T, Vertex> edge)
	{
		_Edges.Remove(edge);

		List<Edge<T, Vertex>>? connectedEdges;
		if (_Vertices.TryGetValue(edge.Vertex1, out connectedEdges) && connectedEdges != null)
			connectedEdges.Remove(edge);
		if (_Vertices.TryGetValue(edge.Vertex2, out connectedEdges) && connectedEdges != null)
			connectedEdges.Remove(edge);

		if (edge.Left != null)
		{
			Triangle<T, Vertex> triangle = edge.Left;
			_Triangles.Remove(triangle);

			foreach (Edge<T, Vertex> otherEdge in triangle.Edges)
			{
				if (triangle.Equals(otherEdge.Left))
					otherEdge.Left = null;
				if (triangle.Equals(otherEdge.Right))
					otherEdge.Right = null;
			}
		}
		if (edge.Right != null)
		{
			Triangle<T, Vertex> triangle = edge.Right;
			_Triangles.Remove(triangle);

			foreach (Edge<T, Vertex> otherEdge in triangle.Edges)
			{
				if (triangle.Equals(otherEdge.Left))
					otherEdge.Left = null;
				if (triangle.Equals(otherEdge.Right))
					otherEdge.Right = null;
			}
		}
	}

	/// <summary>
	/// Removes edges from the mesh, as well as any triangle attached to those edges.
	/// </summary>
	/// <param name="edge">The edges to remove.</param>
	protected void _RemoveEdges(IEnumerable<Edge<T, Vertex>> edges)
	{
		foreach (Edge<T, Vertex> edge in edges)
			_RemoveEdge(edge);
	}

	/// <summary>
	/// References an existing edge.
	/// </summary>
	/// <param name="vertex1">A vertex defining an endpoint of the edge.</param>
	/// <param name="vertex2">A vertex defining an endpoint of the edge.</param>
	/// <returns>An existing edge, or null if one does not exist.</returns>
	internal Edge<T, Vertex>? FindExistingEdge(Vertex vertex1, Vertex vertex2)
	{
		if (_Vertices.TryGetValue(vertex1, out List<Edge<T, Vertex>>? edges) && edges != null)
		{
			return edges.FirstOrDefault(e => e.Vertices.Contains(vertex2));
		}
		return null;
	}

	/// <summary>
	/// References an existing edge, or creates a new one if the edge does not exist.
	/// </summary>
	/// <param name="vertex1">A vertex defining an endpoint of the edge.</param>
	/// <param name="vertex2">A vertex defining an endpoint of the edge.</param>
	/// <returns>An existing or a new edge.</returns>
	internal Edge<T, Vertex> FindOrCreateEdge(Vertex vertex1, Vertex vertex2) => 
		FindExistingEdge(vertex1, vertex2) ?? new Edge<T, Vertex>(vertex1, vertex2);

	/// <summary>
	/// Locates all edges associated with the given vertex, sorted in counter-clockwise order.
	/// </summary>
	/// <param name="vertex">A vertex.</param>
	/// <returns>The edges associated with the given vertex, sorted in counter-clockwise order.</returns>
	internal ImmutableList<Edge<T, Vertex>> FindAllEdges(Vertex vertex)
	{
		if (_Vertices.TryGetValue(vertex, out List<Edge<T, Vertex>>? edges) && edges != null)
		{
			return edges.ToImmutableList();
		}
		return new List<Edge<T, Vertex>>().ToImmutableList();
	}

	#endregion Methods
}
