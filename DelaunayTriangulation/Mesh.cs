using System;
using System.Numerics;

namespace DelaunayTriangulation;

public class Mesh<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	/// <summary>
	/// Tolerance for saying a number is approximately equal to another number.
	/// </summary>
	public static T NumericTolerance = T.CreateChecked(1E-10);

	/// <summary>
	/// The vertices of the mesh.
	/// </summary>
	private HashSet<Vertex> _Vertices = new HashSet<Vertex>();

	/// <summary>
	/// The vertices of the mesh.
	/// </summary>
	public IEnumerable<Vertex> Vertices => _Vertices;

	/// <summary>
	/// The edges of the mesh.
	/// </summary>
	private HashSet<Edge<T, Vertex>> _Edges = new HashSet<Edge<T, Vertex>>();

	/// <summary>
	/// The edges of the convex hull.
	/// </summary>
	private List<Edge<T, Vertex>> _ConvexHull;

	/// <summary>
	/// The triangles of the mesh.
	/// </summary>
	private HashSet<Triangle<T, Vertex>> _Triangles = new HashSet<Triangle<T, Vertex>>();

	/// <summary>
	/// The triangles of the mesh.
	/// </summary>
	public IEnumerable<Triangle<T, Vertex>> Triangles => _Triangles;

	/// <summary>
	/// References an existing edge, or creates a new one if the edge does not exist.
	/// </summary>
	/// <param name="vertex1">A vertex defining an endpoint of the edge.</param>
	/// <param name="vertex2">A vertex defining an endpoint of the edge.</param>
	/// <returns>An existing or a new edge.</returns>
	internal Edge<T, Vertex> FindOrCreateEdge(Vertex vertex1, Vertex vertex2)
	{
		return _Edges.FirstOrDefault(edge => (edge.Vertex1.Equals(vertex1) && edge.Vertex2.Equals(vertex2)) || (edge.Vertex2.Equals(vertex1) && edge.Vertex1.Equals(vertex2))) ?? new Edge<T, Vertex> { Vertex1 = vertex1, Vertex2 = vertex2 };
	}

	#region

	/// <summary>
	/// Constructs a Delaunay triangulation of the given vertices.
	/// </summary>
	/// <param name="vertices">The vertices of a graph.</param>
	/// <exception cref="ArgumentException">Thrown if less than 3 vertices are passed in to the constructor.</exception>
	public Mesh(IEnumerable<Vertex> vertices)
	{
		if (vertices.Count() < 3)
			throw new ArgumentException("Less than 3 vertices were enumerated.");

		// Construct the convex hull
		_ConstructConvexHull(vertices);

		// Construct Delaunay triangulation of convex hull
		List<Edge<T, Vertex>> baseEdgeQueue = new List<Edge<T, Vertex>>
		{
			_Edges.First()
		};
		while (baseEdgeQueue.Count > 0)
		{
			Edge<T, Vertex> baseEdge = baseEdgeQueue[0];
			baseEdgeQueue.RemoveAt(0);

			foreach (Vertex vertex in _Vertices)
			{
				if (vertex.Equals(baseEdge.Vertex1) || vertex.Equals(baseEdge.Vertex2))
					continue;

				Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, baseEdge, vertex);
				if (!_Vertices.Except(triangle.Vertices).Any(v => triangle.IsInsideCircumcircle(v)))
				{
					_Triangles.Add(triangle);
					IEnumerable<Edge<T, Vertex>> newEdges = triangle.Edges.Except(_Edges);
					baseEdgeQueue.AddRange(newEdges);
					_Edges.UnionWith(newEdges);
					triangle.UpdateAdjacentEdges();
					break;
				}
			}
		}

		// Insert the remaining vertices
		foreach (Vertex vertex in vertices.Except(_Vertices))
		{
			_AddInsideConvexHull(vertex);
		}
	}

	#endregion

	#region Convex hull

	/// <summary>
	/// Constructs (or reconstructs) the convex hull of the mesh.
	/// </summary>
	/// <param name="vertices">All vertices that will eventually be part of the mesh.</param>
	/// <exception cref="ArgumentException">Thrown if vertices does not enumerate any vertices.</exception>
	private void _ConstructConvexHull(IEnumerable<Vertex> vertices)
	{
		if (!vertices.Any())
			throw new ArgumentException("No vertices.");
		_ConvexHull = new List<Edge<T, Vertex>>();

		// Construct set of initial vertices
		(Vertex, T) minA = (vertices.First(), vertices.First().X + vertices.First().Y);
		(Vertex, T) maxA = (vertices.First(), vertices.First().X + vertices.First().Y);
		(Vertex, T) minB = (vertices.First(), vertices.First().X - vertices.First().Y);
		(Vertex, T) maxB = (vertices.First(), vertices.First().X - vertices.First().Y);
		foreach (Vertex vertex in vertices.Skip(1))
		{
			T a = vertex.X + vertex.Y;
			T b = vertex.X - vertex.Y;
			if (a < minA.Item2)
				minA = (vertex, a);
			if (a > maxA.Item2)
				maxA = (vertex, a);
			if (b < minB.Item2)
				minB = (vertex, b);
			if (b > maxB.Item2)
				maxB = (vertex, b);
		}
		List<Vertex> initialVertices = new List<Vertex>(new Vertex[] { maxA.Item1, minB.Item1, minA.Item1, maxB.Item1 }.Distinct());
		_Vertices.UnionWith(initialVertices);

		// Recursion to fill out convex hull
		if (initialVertices.Count > 1)
		{
			for (int k = 0; k < initialVertices.Count; ++k)
			{
				Vertex vertex1 = initialVertices[k];
				Vertex vertex2 = initialVertices[(k + 1) % initialVertices.Count];
				Edge<T, Vertex> edge = FindOrCreateEdge(vertex1, vertex2);
				_ConvexHull.Add(edge);
				_ConstructConvexHullHelper(vertices.Except(_Vertices));
			}
		}
		_Edges.UnionWith(_ConvexHull);
	}

	/// <summary>
	/// Recursive function to find vertices that belong to the convex hull.
	/// </summary>
	/// <param name="vertices">Enumerates the remaining vertices to inspect.</param>
	private void _ConstructConvexHullHelper(IEnumerable<Vertex> vertices)
	{
		Edge<T, Vertex> edge = _ConvexHull[_ConvexHull.Count - 1];

		// Find the vertex with the largest righthand offset from this edge
		(Vertex, T)? bestCandidate = null;
		foreach (Vertex vertex in vertices)
		{
			T offset = edge.GetRighthandOffset(vertex);
			if (offset > NumericTolerance)
			{
				if (bestCandidate == null || bestCandidate.Value.Item2 < offset)
				{
					bestCandidate = (vertex, offset);
				}
			}
			else if (offset <= NumericTolerance && offset >= -NumericTolerance)
			{
				Vector2<T> edgeVector = edge.Vector;
				T edgeVectorDot = edgeVector.Dot(Vector2<T>.VectorDifference(edge.Vertex1, vertex));
				if (edgeVectorDot > -NumericTolerance && edgeVectorDot < edgeVector.LengthSquared)
				{
					if (bestCandidate == null || bestCandidate.Value.Item2 < offset)
					{
						bestCandidate = (vertex, offset);
					}
				}
			}
		}

		if (bestCandidate != null)
		{
			// Remove the former edge of the convex hull
			_ConvexHull.RemoveAt(_ConvexHull.Count - 1);

			// Add the candidate vertex
			Vertex vertex = bestCandidate.Value.Item1;
			_Vertices.Add(vertex);

			// Add two edges from each endpoint to the new vertex
			Edge<T, Vertex> edge1 = FindOrCreateEdge(edge.Vertex1, vertex);
			_ConvexHull.Add(edge1);
			_ConstructConvexHullHelper(vertices.Except(_Vertices));

			Edge<T, Vertex> edge2 = FindOrCreateEdge(vertex, edge.Vertex2);
			_ConvexHull.Add(edge2);
			_ConstructConvexHullHelper(vertices.Except(_Vertices));
		}
	}

	/// <summary>
	/// Checks if a vertex is inside the convex hull of the mesh.
	/// </summary>
	/// <param name="vertex">A vertex to test.</param>
	/// <returns>+1 if the vertex is inside the convex hull.
	/// 0 if the vertex is on the convex hull.
	/// -1 if the vertex is outside the convex hull.</returns>
	public int IsInsideConvexHull(Vertex vertex)
	{
		List<T> offsets = new List<T>(_ConvexHull.Select(edge => edge.GetRighthandOffset(vertex)));
		if (offsets.Any(offset => offset > NumericTolerance))
			return 1;
		else if (offsets.All(offset => offset < -NumericTolerance))
			return -1;
		else
			return 0;
	}

	#endregion Convex hull

	#region Insertion

	/// <summary>
	/// Inserts a vertex which is known to exist strictly inside the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist strictly inside the convex hull.</param>
	private void _AddInsideConvexHull(Vertex vertex)
	{

	}

	/// <summary>
	/// Inserts a vertex which is known to exist on the boundary of the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist on the boundary of the convex hull.</param>
	private void _AddOnBoundaryOfConvexHull(Vertex vertex)
	{

	}

	/// <summary>
	/// Inserts a vertex which is known to exist strictly outside the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist strictly outside the convex hull.</param>
	private void _AddOutsideConvexHull(Vertex vertex)
	{

	}

	/// <summary>
	/// Combines this mesh in-place with a mesh that has no overlap of the convex hull.
	/// </summary>
	/// <param name="mesh">The mesh to combine with.</param>
	private void _Merge(Mesh<T, Vertex> mesh)
	{

	}

	/// <summary>
	/// Adds a single vertex to the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to insert.</param>
	public void Add(Vertex vertex)
	{
		switch (IsInsideConvexHull(vertex))
		{
			case 1:
				_AddInsideConvexHull(vertex);
				break;
			case -1:
				_AddOutsideConvexHull(vertex);
				break;
			default:
				_AddOnBoundaryOfConvexHull(vertex);
				break;
		}
	}

	public void AddRange(IEnumerable<Vertex> vertices)
	{
		// Insert all of the vertices which are known to exist inside or on the boundary of the convex hull of this mesh
		List<Vertex> remainingVertices = new List<Vertex>(vertices);
		int index = 0;
		int numInspectedSinceLastUpdateToConvexHull = 0;
		while (numInspectedSinceLastUpdateToConvexHull < remainingVertices.Count)
		{
			Vertex vertex = remainingVertices[index];
			++numInspectedSinceLastUpdateToConvexHull;

			int region = IsInsideConvexHull(vertex);
			if (region > 0)
			{
				_AddInsideConvexHull(vertex);
				remainingVertices.RemoveAt(index);
			}
			else if (region == 0)
			{
				_AddOnBoundaryOfConvexHull(vertex);
				remainingVertices.RemoveAt(index);
				numInspectedSinceLastUpdateToConvexHull = 0;
			}
			else ++index;

			index = index % remainingVertices.Count;
		}

		// Handle the vertices outside of the convex hull
		if (remainingVertices.Count < 3)
		{
			// Number of vertices is too small to construct a separate mesh, so insert the vertices one at a time
			foreach (Vertex vertex in remainingVertices)
				_AddOutsideConvexHull(vertex);
		}
		else
		{
			// Construct a new mesh from the remaining vertices, then merge into this mesh
			_Merge(new Mesh<T, Vertex>(remainingVertices));
		}
	}

	#endregion Insertion

}
