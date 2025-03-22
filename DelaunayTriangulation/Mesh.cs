using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace DelaunayTriangulation;

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
	protected Dictionary<Vertex, List<Edge<T, Vertex>> _Vertices = new Dictionary<Vertex, List<Edge<T, Vertex>>>();

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

	/// <summary>
	/// Adds a single vertex to the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to insert.</param>
	public abstract void Add(Vertex vertex);

	/// <summary>
	/// Adds multiple vertices to this mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertices">The vertices to insert.</param>
	public abstract void AddRange(IEnumerable<Vertex> vertices);

	/// <summary>
	/// Removes a single vertex from the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to remove.</param>
	public abstract void Remove(Vertex vertex);

	/// <summary>
	/// Removes multiple vertices from this mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertices">The vertices to remove.</param>
	public abstract void RemoveRange(IEnumerable<Vertex> vertices);

	#endregion Methods
}
