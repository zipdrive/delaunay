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
	/// The edges of the mesh.
	/// </summary>
	public IEnumerable<Edge<T, Vertex>> Edges => _Edges;

	/// <summary>
	/// The edges of the convex hull.
	/// </summary>
	private List<Edge<T, Vertex>> _ConvexHull = new List<Edge<T, Vertex>>();

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

	#region Initial construction

	/// <summary>
	/// Uses the divide-and-conquer approach to divide the vertices into increasingly-small subsets, then merging the resulting triangulations into the final mesh.
	/// </summary>
	/// <param name="vertices">The vertices of the mesh.</param>
	/// <returns>A Delaunay triangulation of the vertices.</returns>
	public static Mesh<T, Vertex> Construct(IEnumerable<Vertex> vertices) => _ConstructHelperX(vertices.ToArray());

	/// <summary>
	/// Recursive helper function to subdivide the graph into two regions, then merge the resulting triangulations.
	/// </summary>
	/// <param name="vertices">The vertices of the graph.</param>
	/// <returns>The Delaunay triangulation of those vertices.</returns>
	private static Mesh<T, Vertex> _ConstructHelperX(Vertex[] vertices)
	{
		if (vertices.Length > 3)
		{
			Array.Sort(vertices, (lhs, rhs) => lhs.X.CompareTo(rhs.X));
			Mesh<T, Vertex> mesh1 = _ConstructHelperY(vertices[..(vertices.Length / 2)]);
			Mesh<T, Vertex> mesh2 = _ConstructHelperY(vertices[(vertices.Length / 2)..]);
			mesh1._Merge(mesh2);
			return mesh1;
		}
		else
		{
			return new Mesh<T, Vertex>(vertices);
		}
	}

	/// <summary>
	/// Recursive helper function to subdivide the graph into two regions, then merge the resulting triangulations.
	/// </summary>
	/// <param name="vertices">The vertices of the graph.</param>
	/// <returns>The Delaunay triangulation of those vertices.</returns>
	private static Mesh<T, Vertex> _ConstructHelperY(Vertex[] vertices)
	{
		if (vertices.Length > 3)
		{
			Array.Sort(vertices, (lhs, rhs) => lhs.Y.CompareTo(rhs.Y));
			Mesh<T, Vertex> mesh1 = _ConstructHelperY(vertices[..(vertices.Length / 2)]);
			Mesh<T, Vertex> mesh2 = _ConstructHelperY(vertices[(vertices.Length / 2)..]);
			mesh1._Merge(mesh2);
			return mesh1;
		}
		else
		{
			return new Mesh<T, Vertex>(vertices);
		}
	}

	/// <summary>
	/// Constructs a mesh with a single vertex.
	/// </summary>
	/// <param name="vertex">The only vertex in the mesh.</param>
	internal Mesh(Vertex vertex)
	{
		_Vertices = new HashSet<Vertex> { vertex };
		_Edges = new HashSet<Edge<T, Vertex>>();
		_Triangles = new HashSet<Triangle<T, Vertex>>();
	}

	/// <summary>
	/// Constructs a Delaunay triangulation of the given vertices.
	/// </summary>
	/// <param name="vertices">The vertices of a graph.</param>
	/// <exception cref="ArgumentException">Thrown if more than 3 vertices are passed in to the constructor.</exception>
	internal Mesh(IEnumerable<Vertex> vertices)
	{
		if (vertices.Count() > 3)
			throw new ArgumentException("More than 3 vertices passed to Mesh constructor.");

		// Initial vertices
		_Vertices = new HashSet<Vertex>(vertices);
		
		// Initial edges
		foreach (Vertex vertex1 in vertices)
		{
			foreach (Vertex vertex2 in vertices.TakeWhile(v => !v.Equals(vertex1)))
			{
				_Edges.Add(new Edge<T, Vertex> { Vertex1 = vertex1, Vertex2 = vertex2 });
				
				// Initial triangles
				foreach (Vertex vertex3 in vertices.TakeWhile(v => !v.Equals(vertex2)))
				{
					Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, vertex1, vertex2, vertex3);
					_Triangles.Add(triangle);
					
					// Initial convex hull
					_ConvexHull.AddRange(triangle.Edges);
				}
			}
		}

	}

	#endregion Initial construction

	#region Convex hull

	/// <summary>
	/// Constructs (or reconstructs) the convex hull of the mesh.
	/// </summary>
	/// <param name="vertices">All vertices that will eventually be part of the mesh.</param>
	/// <exception cref="ArgumentException">Thrown if vertices does not enumerate any vertices.</exception>
	private void _ConstructConvexHull()
	{
		_ConvexHull = new List<Edge<T, Vertex>>();
		if (_Vertices.Count < 3)
			return;

		// Construct set of initial vertices
		(Vertex, T) minA = (_Vertices.First(), _Vertices.First().X + _Vertices.First().Y);
		(Vertex, T) maxA = (_Vertices.First(), _Vertices.First().X + _Vertices.First().Y);
		(Vertex, T) minB = (_Vertices.First(), _Vertices.First().X - _Vertices.First().Y);
		(Vertex, T) maxB = (_Vertices.First(), _Vertices.First().X - _Vertices.First().Y);
		foreach (Vertex vertex in _Vertices.Skip(1))
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
				if (!edge.Vertex1.Equals(vertex1))
					edge.Flip();

				_ConvexHull.Add(edge);
				_ConstructConvexHullHelper(_Vertices.Except(initialVertices));
			}
		}
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
			IEnumerable<Vertex> remainingVertices = vertices.Where(v => !v.Equals(vertex));

			// Add two edges from each endpoint to the new vertex
			Edge<T, Vertex> edge1 = FindOrCreateEdge(edge.Vertex1, vertex);
			_ConvexHull.Add(edge1);
			_ConstructConvexHullHelper(remainingVertices);

			Edge<T, Vertex> edge2 = FindOrCreateEdge(vertex, edge.Vertex2);
			_ConvexHull.Add(edge2);
			_ConstructConvexHullHelper(remainingVertices);
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
		foreach (Edge<T, Vertex> edge in _ConvexHull)
		{
			T offset = edge.GetRighthandOffset(vertex);
			if (offset > NumericTolerance)
				return -1;
			else if (offset >= -NumericTolerance)
			{
				Vector2<T> edgeVector = edge.Vector;
				T edgeVectorDot = edgeVector.Dot(Vector2<T>.VectorDifference(vertex, edge.Vertex1));
				if (edgeVectorDot >= -NumericTolerance && edgeVectorDot <= edgeVector.LengthSquared + NumericTolerance)
					return 0;
			}
		}
		return 1;
	}

	#endregion Convex hull

	#region Insertion

	/// <summary>
	/// Inserts a vertex which is known to exist strictly inside the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist strictly inside the convex hull.</param>
	private void _AddInsideConvexHull(Vertex vertex)
	{
		#region Construct influence triangulation
		HashSet<Triangle<T, Vertex>> influenceTriangulation = new HashSet<Triangle<T, Vertex>>();
		
		// Iterate over all triangles to see which triangles contain the vertex in circumcenter
		foreach (Triangle<T, Vertex> triangle in _Triangles)
		{
			if (triangle.IsInsideCircumcircle(vertex))
			{
				HashSet<Triangle<T, Vertex>> suspects = new HashSet<Triangle<T, Vertex>> { triangle };
				HashSet<Triangle<T, Vertex>> clearedSuspects = new HashSet<Triangle<T, Vertex>>();
				while (suspects.Count > 0)
				{
					Triangle<T, Vertex> suspectedTriangle = suspects.First();
					suspects.Remove(suspectedTriangle);
					if (influenceTriangulation.Contains(suspectedTriangle) || clearedSuspects.Contains(suspectedTriangle))
						continue;

					if (suspectedTriangle.IsInsideCircumcircle(vertex))
					{
						influenceTriangulation.Add(suspectedTriangle);
						suspects.UnionWith(suspectedTriangle.AdjacentTriangles);
					}
					else
					{
						clearedSuspects.Add(suspectedTriangle);
					}
				}
				break;
			}
		}
		#endregion Construct influence triangulation

		// Remove all triangles in the influence triangulation
		_Triangles.ExceptWith(influenceTriangulation);
		// Remove all edges shared by TWO of the removed triangles (i.e. do not remove the edges on the border of the influence triangulation)
		_Edges.RemoveWhere(e => influenceTriangulation.Count(t => t.Edges.Contains(e)) > 1);

		// Add the new vertex, with triangles connecting it to each edge forming the convex hull of the influence triangulation
		_Vertices.Add(vertex);
		foreach (Edge<T, Vertex> influenceTriangulationConvexHullEdge in _Edges.Where(e => influenceTriangulation.Any(t => t.Edges.Contains(e))))
		{
			Triangle<T, Vertex> newTriangle = new Triangle<T, Vertex>(this, influenceTriangulationConvexHullEdge, vertex);
			_Triangles.Add(newTriangle);
			_Edges.UnionWith(newTriangle.Edges);
			newTriangle.UpdateAdjacentEdges();
		}
	}

	/// <summary>
	/// Inserts a vertex which is known to exist on the boundary of the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist on the boundary of the convex hull.</param>
	private void _AddOnBoundaryOfConvexHull(Vertex vertex)
	{
		_AddInsideConvexHull(vertex);
		_ConstructConvexHull();
	}

	/// <summary>
	/// Combines this mesh in-place with a mesh that has no overlap of the convex hull.
	/// </summary>
	/// <param name="mesh">The mesh to combine with.</param>
	private void _Merge(Mesh<T, Vertex> mesh)
	{
		Console.WriteLine("Merge called.");

		// Get the convex hull vertices from each triangulation
		List<Vertex> convexHullVerticesL = new List<Vertex>(_ConvexHull.Select(e => e.Vertex1));
		List<Vertex> convexHullVerticesR = new List<Vertex>(_ConvexHull.Select(e => e.Vertex1));

		#region Linearly separate the two convex hulls
		// Find a vector parallel to a line that separates the two convex hulls
		Vector2<T>? separatingVector = null;
		
		// First, test the vectors parallel to each convex hull edge on the right mesh
		foreach (Edge<T, Vertex> boundaryEdgeR in mesh._ConvexHull)
		{
			Vector2<T> vector = boundaryEdgeR.Vector;
			if (convexHullVerticesL.All(v => boundaryEdgeR.GetRighthandOffset(v) > NumericTolerance))
			{
				separatingVector = vector;
				break;
			}
		}

		// Then, test the vectors parallel to each convex hull edge on the left mesh
		if (separatingVector == null)
		{
			foreach (Edge<T, Vertex> boundaryEdgeL in _ConvexHull)
			{
				Vector2<T> vector = boundaryEdgeL.Vector;
				if (convexHullVerticesR.All(v => boundaryEdgeL.GetRighthandOffset(v) > NumericTolerance))
				{
					separatingVector = vector;
					break;
				}
			}
		}

		// Throw an error if a vector to separate the two meshes was not found
		if (separatingVector == null)
		{
			if (_Vertices.Count == 0)
			{
				_Vertices = mesh._Vertices;
				_Edges = mesh._Edges;
				_Triangles = mesh._Triangles;
				_ConvexHull = mesh._ConvexHull;
				return;
			}
			else if (_Vertices.Count == 1 && mesh._Vertices.Count == 1)
			{
				_Edges.Add(new Edge<T, Vertex>
				{
					Vertex1 = _Vertices.First(),
					Vertex2 = mesh._Vertices.First()
				});
				_Vertices.Add(mesh._Vertices.First());
				return;
			}
			else if (_Edges.Count > 0)
			{
				Edge<T, Vertex> edge = _Edges.First();
				if (edge.GetRighthandOffset(mesh._Vertices.First()) > NumericTolerance)
					edge.Flip();
				_ConvexHull.Add(edge);

				if (mesh._Edges.Count > 0)
				{
					Edge<T, Vertex> otherEdge = _Edges.First();
					if (otherEdge.GetRighthandOffset(_Vertices.First()) > NumericTolerance)
						otherEdge.Flip();
					_ConvexHull.Add(otherEdge);
				}

				separatingVector = edge.Vector;
			}
			else if (mesh._Edges.Count > 0)
			{
				Edge<T, Vertex> otherEdge = _Edges.First();
				if (otherEdge.GetRighthandOffset(_Vertices.First()) > NumericTolerance)
					otherEdge.Flip();
				_ConvexHull.Add(otherEdge);

				separatingVector = otherEdge.Vector;
			}
			else
			{
				throw new Exception("Could not determine an axis to separate convex hulls for merge operation.");
			}
		}
		#endregion Linearly separate the two convex hulls

		// Start on each side with the vertex farthest in the direction of the vector found previously
		Vertex l = (convexHullVerticesL.Count > 0 ? convexHullVerticesL.AsEnumerable() : _Vertices.AsEnumerable())
			.OrderBy(v => separatingVector.X * v.X + separatingVector.Y * v.Y)
			.First();
		Vertex r = (convexHullVerticesR.Count > 0 ? convexHullVerticesR.AsEnumerable() : mesh._Vertices.AsEnumerable())
			.OrderBy(v => separatingVector.X * v.X + separatingVector.Y * v.Y)
			.First();

		HashSet<Edge<T, Vertex>> edgesLL = new HashSet<Edge<T, Vertex>>(_Edges);

		_Vertices.Add(r);
		Edge<T, Vertex> connectingEdge = new Edge<T, Vertex> { Vertex1 = l, Vertex2 = r };
		_Edges.Add(connectingEdge);

		// Then, iterate
		while (true)
		{
			#region Left candidate vertex
			SortedList<T, Vertex> potentialCandidateVerticesL = new SortedList<T, Vertex>();
			foreach (Edge<T, Vertex> edgeL in edgesLL)
			{
				if (edgeL.Vertex1.Equals(l) || edgeL.Vertex2.Equals(l))
				{
					T edgeAngleL = connectingEdge.GetAngularDifference(edgeL);
					potentialCandidateVerticesL.Add(edgeAngleL, edgeL.Vertex1.Equals(l) ? edgeL.Vertex2 : edgeL.Vertex1);
				}
			}

			(Vertex, Triangle<T, Vertex>)? candidateL = null;
			while (potentialCandidateVerticesL.Count > 0)
			{
				if (potentialCandidateVerticesL.Keys[0] >= T.Pi - NumericTolerance)
				{
					break; // Angle of next potential candidate is greater than or equal to 180 degrees, so no candidate is chosen for left side
				}

				Vertex nextCandidateVertexL = potentialCandidateVerticesL.Values[0];
				potentialCandidateVerticesL.RemoveAt(0);

				Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(this, connectingEdge, nextCandidateVertexL);
				if (potentialCandidateVerticesL.Count > 0)
				{
					if (triangle.IsInsideCircumcircle(potentialCandidateVerticesL.Values[0]))
					{
						Edge<T, Vertex>? contradictedEdgeL = FindOrCreateEdge(l, nextCandidateVertexL);
						if (contradictedEdgeL == null)
							throw new Exception("Expected to find an edge in the triangulation which does not exist.");
						_RemoveEdge(contradictedEdgeL);
						edgesLL.Remove(contradictedEdgeL);
					}
					else
					{
						candidateL = (nextCandidateVertexL, triangle);
						break;
					}
				}
				else candidateL = (nextCandidateVertexL, triangle);
			}
			#endregion Left candidate vertex

			#region Right candidate vertex
			SortedList<T, Vertex> potentialCandidateVerticesR = new SortedList<T, Vertex>();
			foreach (Edge<T, Vertex> edgeR in mesh._Edges)
			{
				if (edgeR.Vertex1.Equals(r) || edgeR.Vertex2.Equals(r))
				{
					T edgeAngleR = T.Tau - connectingEdge.GetAngularDifference(edgeR);
					potentialCandidateVerticesR.Add(edgeAngleR, edgeR.Vertex1.Equals(r) ? edgeR.Vertex2 : edgeR.Vertex1);
				}
			}

			(Vertex, Triangle<T, Vertex>)? candidateR = null;
			while (potentialCandidateVerticesR.Count > 0)
			{
				if (potentialCandidateVerticesR.Keys[0] >= T.Pi - NumericTolerance)
				{
					break; // Angle of next potential candidate is greater than or equal to 180 degrees, so no candidate is chosen for left side
				}

				Vertex nextCandidateVertexR = potentialCandidateVerticesR.Values[0];
				potentialCandidateVerticesR.RemoveAt(0);

				Triangle<T, Vertex> triangle = new Triangle<T, Vertex>(mesh, connectingEdge, nextCandidateVertexR);
				if (potentialCandidateVerticesR.Count > 0)
				{
					if (triangle.IsInsideCircumcircle(potentialCandidateVerticesR.Values[0]))
					{
						Edge<T, Vertex>? contradictedEdgeR = FindOrCreateEdge(l, nextCandidateVertexR);
						if (contradictedEdgeR == null)
							throw new Exception("Expected to find an edge in the triangulation which does not exist.");
						mesh._RemoveEdge(contradictedEdgeR);
					}
					else
					{
						candidateR = (nextCandidateVertexR, triangle);
						break;
					}
				}
				else candidateR = (nextCandidateVertexR, triangle);
			}
			#endregion Right candidate vertex

			#region Insert the vertex, edge, and triangle into this triangulation
			(Vertex, Triangle<T, Vertex>) candidate;
			if (candidateL != null && candidateR != null)
			{
				if (candidateL.Value.Item2.IsInsideCircumcircle(candidateR.Value.Item1))
				{
					candidate = candidateR.Value;
					r = candidateR.Value.Item1;
				}
				else
				{
					candidate = candidateL.Value;
					l = candidateL.Value.Item1;
				}
			}
			else if (candidateL != null)
			{
				candidate = candidateL.Value;
				l = candidateL.Value.Item1;
			}
			else if (candidateR != null)
			{
				candidate = candidateR.Value;
				r = candidateR.Value.Item1;
			}
			else
				break;

			_Vertices.Add(candidate.Item1);
			_Edges.UnionWith(candidate.Item2.Edges);
			_Triangles.Add(candidate.Item2);

			connectingEdge = FindOrCreateEdge(l, r);
			#endregion Insert the vertex, edge, and triangle into this triangulation
		}

		// Insert all vertices and all remaining edges/triangles from the other triangulation
		_Vertices.UnionWith(mesh._Vertices);
		_Edges.UnionWith(mesh._Edges);
		_Triangles.UnionWith(mesh._Triangles);

		// Recalculate edge adjacencies
		foreach (Triangle<T, Vertex> triangle in Triangles)
		{
			triangle.UpdateAdjacentEdges();
		}

		// Recalculate convex hull
		_ConstructConvexHull();
	}

	/// <summary>
	/// Adds a single vertex to the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to insert.</param>
	public void Add(Vertex vertex)
	{
		int region = IsInsideConvexHull(vertex);
		if (region > 0)
		{
			_AddInsideConvexHull(vertex);
		}
		else if (region < 0)
		{
			// Merge a mesh with a single vertex into this one
			_Merge(new Mesh<T, Vertex>(vertex));
		}
		else
		{
			_AddOnBoundaryOfConvexHull(vertex);
		}
	}

	/// <summary>
	/// Adds multiple vertices to this mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertices">The vertices to insert.</param>
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
		if (remainingVertices.Count > 0)
		{
			// Construct a new mesh from the remaining vertices, then merge into this mesh
			_Merge(Construct(remainingVertices));
		}
	}

	#endregion Insertion

	#region Removal

	/// <summary>
	/// Removes an edge and any triangles associated with that edge.
	/// </summary>
	/// <param name="edge">The edge to remove.</param>
	private void _RemoveEdge(Edge<T, Vertex> edge)
	{
		_Edges.Remove(edge);
		_ConvexHull.Remove(edge);
		_Triangles.RemoveWhere(triangle => triangle.Edges.Contains(edge));
	}

	/// <summary>
	/// Removes a single vertex from the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to be removed.</param>
	public void Remove(Vertex vertex)
	{

	}

	/// <summary>
	/// Removes multiple vertices from the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertices">The vertices to be removed.</param>
	public void RemoveRange(IEnumerable<Vertex> vertices)
	{

	}

	#endregion Removal

}
