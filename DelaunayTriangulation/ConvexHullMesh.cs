using System;
using System.Collections.Generic;
using System.Numerics;

namespace DelaunayTriangulation;

/// <summary>
/// A Delaunay triangulation where the boundary is the convex hull of the vertices.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
/// <typeparam name="Vertex">The data type for each vertex.</typeparam>
public class ConvexHullMesh<T, Vertex> : Mesh<T, Vertex> where T : IFloatingPointIeee754<T> where Vertex : IVertex2<T>
{
	#region Properties

	/// <summary>
	/// The edges of the convex hull.
	/// </summary>
	private List<Edge<T, Vertex>> _ConvexHull = new List<Edge<T, Vertex>>();

	#endregion Properties

	#region Edge searching

	/// <summary>
	/// References an existing edge.
	/// </summary>
	/// <param name="vertex1">A vertex defining an endpoint of the edge.</param>
	/// <param name="vertex2">A vertex defining an endpoint of the edge.</param>
	/// <returns>An existing edge, or null if one does not exist.</returns>
	internal override Edge<T, Vertex>? FindExistingEdge(Vertex vertex1, Vertex vertex2)
	{
		return _Edges.FirstOrDefault(edge => (edge.Vertex1.Equals(vertex1) && edge.Vertex2.Equals(vertex2)) || (edge.Vertex2.Equals(vertex1) && edge.Vertex1.Equals(vertex2)));
	}

	/// <summary>
	/// References an existing edge, or creates a new one if the edge does not exist.
	/// </summary>
	/// <param name="vertex1">A vertex defining an endpoint of the edge.</param>
	/// <param name="vertex2">A vertex defining an endpoint of the edge.</param>
	/// <returns>An existing or a new edge.</returns>
	internal override Edge<T, Vertex> FindOrCreateEdge(Vertex vertex1, Vertex vertex2)
	{
		return _Edges.FirstOrDefault(edge => (edge.Vertex1.Equals(vertex1) && edge.Vertex2.Equals(vertex2)) || (edge.Vertex2.Equals(vertex1) && edge.Vertex1.Equals(vertex2))) ?? new Edge<T, Vertex>(vertex1, vertex2);
	}

	#endregion Edge searching

	#region Initial construction

	/// <summary>
	/// Uses the divide-and-conquer approach to divide the vertices into increasingly-small subsets, then merging the resulting triangulations into the final mesh.
	/// </summary>
	/// <param name="vertices">The vertices of the mesh.</param>
	/// <returns>A Delaunay triangulation of the vertices.</returns>
	public static ConvexHullMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices) => Construct(vertices, T.Epsilon * T.CreateChecked(4));

	/// <summary>
	/// Uses the divide-and-conquer approach to divide the vertices into increasingly-small subsets, then merging the resulting triangulations into the final mesh.
	/// </summary>
	/// <param name="vertices">The vertices of the mesh.</param>
	/// <param name="numericTolerance">The largest number which is still considered 0.</param>
	/// <returns>A Delaunay triangulation of the vertices.</returns>
	public static ConvexHullMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices, T numericTolerance) => _ConstructHelperX(vertices.ToArray(), numericTolerance);

	/// <summary>
	/// Recursive helper function to subdivide the graph into two regions, then merge the resulting triangulations.
	/// </summary>
	/// <param name="vertices">The vertices of the graph.</param>
	/// <returns>The Delaunay triangulation of those vertices.</returns>
	private static ConvexHullMesh<T, Vertex> _ConstructHelperX(Vertex[] vertices, T numericTolerance)
	{
		if (vertices.Length > 3)
		{
			Array.Sort(vertices, (lhs, rhs) => lhs.X.CompareTo(rhs.X));
			ConvexHullMesh<T, Vertex> mesh1 = _ConstructHelperY(vertices[..(vertices.Length / 2)], numericTolerance);
			ConvexHullMesh<T, Vertex> mesh2 = _ConstructHelperY(vertices[(vertices.Length / 2)..], numericTolerance);
			mesh1._Merge(mesh2);
			return mesh1;
		}
		else
		{
			return new ConvexHullMesh<T, Vertex>(vertices, numericTolerance);
		}
	}

	/// <summary>
	/// Recursive helper function to subdivide the graph into two regions, then merge the resulting triangulations.
	/// </summary>
	/// <param name="vertices">The vertices of the graph.</param>
	/// <returns>The Delaunay triangulation of those vertices.</returns>
	private static ConvexHullMesh<T, Vertex> _ConstructHelperY(Vertex[] vertices, T numericTolerance)
	{
		if (vertices.Length > 3)
		{
			Array.Sort(vertices, (lhs, rhs) => lhs.Y.CompareTo(rhs.Y));
			ConvexHullMesh<T, Vertex> mesh1 = _ConstructHelperY(vertices[..(vertices.Length / 2)], numericTolerance);
			ConvexHullMesh<T, Vertex> mesh2 = _ConstructHelperY(vertices[(vertices.Length / 2)..], numericTolerance);
			mesh1._Merge(mesh2);
			return mesh1;
		}
		else
		{
			return new ConvexHullMesh<T, Vertex>(vertices, numericTolerance);
		}
	}

	/// <summary>
	/// Constructs a mesh with a single vertex.
	/// </summary>
	/// <param name="vertex">The only vertex in the mesh.</param>
	internal ConvexHullMesh(Vertex vertex, T numericTolerance) : base(numericTolerance)
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
	internal ConvexHullMesh(IEnumerable<Vertex> vertices, T numericTolerance) : base(numericTolerance)
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
				_Edges.Add(new Edge<T, Vertex>(vertex1, vertex2));
				
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
	/// Verifies the integrity of the convex hull. Throws an exception if there is a problem.
	/// </summary>
	private void _VerifyConvexHull()
	{
		if (_ConvexHull.Count == 0)
		{
			if (_Vertices.Count >= 3)
				throw new Exception("No convex hull despite having at least three vertices.");
			return;
		}

		for (int k = 0; k < _ConvexHull.Count; ++k)
		{
			Edge<T, Vertex> edge = _ConvexHull[k];
			
			Edge<T, Vertex> nextEdge = _ConvexHull[(k + 1) % _ConvexHull.Count];
			if (!edge.Vertex2.Equals(nextEdge.Vertex1))
				throw new Exception("Discontinuity in the convex hull."); // Discontinuity

			if (_Vertices.Any(vertex => edge.GetRighthandOffset(vertex) > NumericTolerance))
				throw new Exception("Not all vertices in the graph are on the lefthand side of the convex hull edges."); // Not all vertices in graph are on lefthand side
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
		List<Edge<T, Vertex>> influenceTriangulationConvexHull = new List<Edge<T, Vertex>>(_Edges.Where(e => influenceTriangulation.Any(t => t.Edges.Contains(e))));
		foreach (Edge<T, Vertex> influenceTriangulationConvexHullEdge in influenceTriangulationConvexHull)
		{
			Triangle<T, Vertex> newTriangle = new Triangle<T, Vertex>(this, influenceTriangulationConvexHullEdge, vertex);
			_Triangles.Add(newTriangle);
			_Edges.UnionWith(newTriangle.Edges);
			newTriangle.UpdateAdjacentEdges(NumericTolerance);
		}
	}

	/// <summary>
	/// Inserts a vertex which is known to exist on the boundary of the convex hull.
	/// </summary>
	/// <param name="vertex">A vertex which is known to exist on the boundary of the convex hull.</param>
	private void _AddOnBoundaryOfConvexHull(Vertex vertex)
	{
		_AddInsideConvexHull(vertex);

		// Locate the convex hull edge(s) that should be removed
		int convexHullRemovedEdgeIndex = _ConvexHull.FindIndex(edge =>
		{
            // Test if vertex is on the line defined by the edge
            T righthandOffset = edge.GetRighthandOffset(vertex);
            if (righthandOffset >= -NumericTolerance && righthandOffset <= NumericTolerance)
            {
                // Test if vertex is contained in the line segment defined by the edge
                Vector2<T> edgeVector = edge.Vector;
                T dot = edgeVector.Dot(Vector2<T>.VectorDifference(edge.Vertex1, vertex));
                if (dot >= -NumericTolerance && dot <= edgeVector.LengthSquared)
                {
					return true;
                }
            }
			return false;
        });
        if (convexHullRemovedEdgeIndex < 0)
            throw new Exception("Expected a convex hull edge to be removed.");

        Edge<T, Vertex> convexHullRemovedEdge = _ConvexHull[convexHullRemovedEdgeIndex];
		_RemoveEdge(convexHullRemovedEdge);
		_ConvexHull.RemoveAt(convexHullRemovedEdgeIndex);

		// Insert the first edge
		Edge<T, Vertex> edge1 = FindOrCreateEdge(convexHullRemovedEdge.Vertex1, vertex);
		if (!edge1.Vertex2.Equals(vertex))
			edge1.Flip();
		_ConvexHull.Insert(convexHullRemovedEdgeIndex, edge1);

		// Insert the second edge
		Edge<T, Vertex> edge2 = FindOrCreateEdge(vertex, convexHullRemovedEdge.Vertex2);
		if (!edge2.Vertex1.Equals(vertex))
			edge2.Flip();
		_ConvexHull.Insert(convexHullRemovedEdgeIndex + 1, edge2);
	}

	/// <summary>
	/// Search for the edge on the convex hull of the combination of this mesh and another mesh,
	/// where Vertex1 is a vertex of this mesh and Vertex2 is a vertex of the other mesh.
	/// </summary>
	/// <param name="mesh">The other mesh.</param>
	/// <param name="edge">The edge to inspect.</param>
	/// <returns>An edge on the convex hull that bridges this mesh with the other mesh. Null if no such edge was found.</returns>
	private Edge<T, Vertex>? _FindBaseLREdgeHelper(ConvexHullMesh<T, Vertex> mesh, Edge<T, Vertex> edge, HashSet<Vertex> remainingVertices)
	{
        // Find the vertex with the largest righthand offset from this edge
        (Vertex, T)? bestCandidate = null;
        foreach (Vertex vertex in remainingVertices)
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
                if (edgeVectorDot >= -NumericTolerance && edgeVectorDot < edgeVector.LengthSquared)
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
			Vertex bestVertex = bestCandidate.Value.Item1;
			remainingVertices.Remove(bestVertex);

            // Add two edges from each endpoint to the new vertex
            Edge <T, Vertex> edge1 = new Edge<T, Vertex>(edge.Vertex1, bestVertex);
            Edge<T, Vertex> edge2 = new Edge<T, Vertex>(bestVertex, edge.Vertex2);
            return _FindBaseLREdgeHelper(mesh, edge1, remainingVertices) ?? _FindBaseLREdgeHelper(mesh, edge2, remainingVertices);
        }
		else 
		{
			return _Vertices.Contains(edge.Vertex1) && mesh._Vertices.Contains(edge.Vertex2) ? edge : null;
		}
    }

    /// <summary>
    /// Search for the edge on the convex hull of the combination of this mesh and another mesh,
    /// where Vertex1 is a vertex of this mesh and Vertex2 is a vertex of the other mesh.
    /// </summary>
    /// <param name="mesh">The other mesh.</param>
    /// <returns>An edge on the convex hull that bridges this mesh with the other mesh. Null if no such edge was found.</returns>
    private Edge<T, Vertex> _FindBaseLREdge(ConvexHullMesh<T, Vertex> mesh)
	{
        // Construct set of initial vertices
        (Vertex, T) minA = (_Vertices.First(), _Vertices.First().X + _Vertices.First().Y);
        (Vertex, T) maxA = (_Vertices.First(), _Vertices.First().X + _Vertices.First().Y);
        (Vertex, T) minB = (_Vertices.First(), _Vertices.First().X - _Vertices.First().Y);
        (Vertex, T) maxB = (_Vertices.First(), _Vertices.First().X - _Vertices.First().Y);
        foreach (Vertex vertex in _Vertices.Skip(1).Concat(mesh._Vertices))
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
        
        // Recursion to fill out convex hull
        if (initialVertices.Count > 1)
        {
			HashSet<Vertex> remainingVertices = new HashSet<Vertex>(_Vertices.Concat(mesh._Vertices));
			remainingVertices.ExceptWith(initialVertices);

            for (int k = 0; k < initialVertices.Count; ++k)
            {
                Vertex vertex1 = initialVertices[k];
                Vertex vertex2 = initialVertices[(k + 1) % initialVertices.Count];

                Edge<T, Vertex> edge = new Edge<T, Vertex>(vertex1, vertex2);
                Edge<T, Vertex>? result = _FindBaseLREdgeHelper(mesh, edge, remainingVertices);
				if (result != null)
					return result;
            }
        }

		_VerifyConvexHull();
		throw new Exception("Could not find a base LR edge to merge two meshes.");
    }

	/// <summary>
	/// Combines this mesh in-place with a mesh that has no overlap of the convex hull.
	/// </summary>
	/// <param name="mesh">The mesh to combine with.</param>
	private void _Merge(ConvexHullMesh<T, Vertex> mesh)
	{
        // Get the LL edges
        HashSet<Edge<T, Vertex>> edgesLL = new HashSet<Edge<T, Vertex>>(_Edges);

        // Get the convex hull vertices from each triangulation
        List<Vertex> convexHullVerticesL = new List<Vertex>(_ConvexHull.Count > 0 ? _ConvexHull.Select(e => e.Vertex1) : _Vertices);
		List<Vertex> convexHullVerticesR = new List<Vertex>(mesh._ConvexHull.Count > 0 ? mesh._ConvexHull.Select(e => e.Vertex1) : mesh._Vertices);

		// Create the first connecting edge between the two meshes
		Edge<T, Vertex> convexHullLREdge = _FindBaseLREdge(mesh);
		Edge<T, Vertex> connectingEdge = convexHullLREdge;
		Vertex l = connectingEdge.Vertex1;
		Vertex r = connectingEdge.Vertex2;
		_Vertices.Add(r);
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
						Edge<T, Vertex> contradictedEdgeL = _RemoveEdge(l, nextCandidateVertexL);
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
						mesh._RemoveEdge(r, nextCandidateVertexR);
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
				else //if (candidateR.Value.Item2.IsInsideCircumcircle(candidateL.Value.Item1))
				{
					// If both vertices are on the same circumcircle (within tolerance), we arbitrarily pick the left vertex as our candidate
					candidate = candidateL.Value;
					l = candidateL.Value.Item1;
				}
				//else throw new Exception("The circumcircles of both candidate vertices contain the other candidate.");
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

			connectingEdge = FindOrCreateEdge(r, l);
			if (!connectingEdge.Vertex1.Equals(r))
				connectingEdge.Flip();
			#endregion Insert the vertex, edge, and triangle into this triangulation
		}

		Edge<T, Vertex> convexHullRLEdge = connectingEdge;

		// Insert all vertices and all remaining edges/triangles from the other triangulation
		_Vertices.UnionWith(mesh._Vertices);
		_Edges.UnionWith(mesh._Edges);
		_Triangles.UnionWith(mesh._Triangles);

		// Recalculate edge adjacencies
		foreach (Triangle<T, Vertex> triangle in Triangles)
		{
			triangle.UpdateAdjacentEdges(NumericTolerance);
		}

        // Recalculate convex hull
        Dictionary<Vertex, Edge<T, Vertex>> convexHullLL = new Dictionary<Vertex, Edge<T, Vertex>>(_ConvexHull.Select(e => new KeyValuePair<Vertex, Edge<T, Vertex>>(e.Vertex1, e)));
        Dictionary<Vertex, Edge<T, Vertex>> convexHullRR = new Dictionary<Vertex, Edge<T, Vertex>>(mesh._ConvexHull.Select(e => new KeyValuePair<Vertex, Edge<T, Vertex>>(e.Vertex1, e)));

        _ConvexHull = new List<Edge<T, Vertex>> { convexHullRLEdge };

		Vertex lastConvexHullVertex = convexHullRLEdge.Vertex2;
		while (!lastConvexHullVertex.Equals(convexHullLREdge.Vertex1))
		{
			if (convexHullLL.Count > 0)
			{
				if (convexHullLL.TryGetValue(lastConvexHullVertex, out Edge<T, Vertex>? nextConvexHullEdge) && nextConvexHullEdge != null)
				{
					_ConvexHull.Add(nextConvexHullEdge);
					lastConvexHullVertex = nextConvexHullEdge.Vertex2;
				}
				else throw new Exception("Expected to find edge on the convex hull that does not exist.");
			}
			else
			{
				Edge<T, Vertex>? nextConvexHullEdge = edgesLL.FirstOrDefault(e => e.Vertex1.Equals(lastConvexHullVertex) || e.Vertex2.Equals(lastConvexHullVertex));
				if (nextConvexHullEdge == null)
					throw new Exception("Expected to find edge that does not exist.");
				else
				{
					if (nextConvexHullEdge.Vertex2.Equals(lastConvexHullVertex))
						nextConvexHullEdge.Flip();
					_ConvexHull.Add(nextConvexHullEdge);
					lastConvexHullVertex = nextConvexHullEdge.Vertex2;
				}
			}
		}

		_ConvexHull.Add(convexHullLREdge);

        lastConvexHullVertex = convexHullLREdge.Vertex2;
        while (!lastConvexHullVertex.Equals(convexHullRLEdge.Vertex1))
        {
			if (convexHullRR.Count > 0)
			{
				if (convexHullRR.TryGetValue(lastConvexHullVertex, out Edge<T, Vertex>? nextConvexHullEdge) && nextConvexHullEdge != null)
				{
					_ConvexHull.Add(nextConvexHullEdge);
					lastConvexHullVertex = nextConvexHullEdge.Vertex2;
				}
				else throw new Exception("Expected to find edge on the convex hull that does not exist.");
			}
			else
			{
                Edge<T, Vertex>? nextConvexHullEdge = mesh._Edges.FirstOrDefault(e => e.Vertex1.Equals(lastConvexHullVertex) || e.Vertex2.Equals(lastConvexHullVertex));
                if (nextConvexHullEdge == null)
                    throw new Exception("Expected to find edge that does not exist.");
                else
                {
                    if (nextConvexHullEdge.Vertex2.Equals(lastConvexHullVertex))
                        nextConvexHullEdge.Flip();
                    _ConvexHull.Add(nextConvexHullEdge);
                    lastConvexHullVertex = nextConvexHullEdge.Vertex2;
                }
            }
        }
	}

	/// <summary>
	/// Adds a single vertex to the mesh, re-updating the triangulation to compensate.
	/// </summary>
	/// <param name="vertex">The vertex to insert.</param>
	public override void Add(Vertex vertex)
	{
		int region = IsInsideConvexHull(vertex);
		if (region > 0)
		{
			_AddInsideConvexHull(vertex);
		}
		else if (region < 0)
		{
			// Merge a mesh with a single vertex into this one
			_Merge(new ConvexHullMesh<T, Vertex>(vertex, NumericTolerance));
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
	public override void AddRange(IEnumerable<Vertex> vertices)
	{
		// Insert all of the vertices which are known to exist inside or on the boundary of the convex hull of this mesh
		List<Vertex> remainingVertices = new List<Vertex>(vertices);
        while (remainingVertices.Count > 0)
        {
			// Handle vertices inside the convex hull
            int index = 0;
			int numLeftToInspect = remainingVertices.Count;
			while (numLeftToInspect > 0)
			{
				Vertex vertex = remainingVertices[index];
				--numLeftToInspect;

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
					numLeftToInspect = remainingVertices.Count;
				}
				else ++index;

				if (index >= remainingVertices.Count)
					index = 0;
			}

			if (remainingVertices.Count > 0)
			{
				// Handle the vertices outside of the convex hull
				bool anyVerticesAddedToMesh = false;
				foreach (Edge<T, Vertex> convexHullEdge in _ConvexHull)
				{
					List<Vertex> partitionedVertices = new List<Vertex>();
					for (int i = remainingVertices.Count - 1; i >= 0; --i)
					{
						if (convexHullEdge.GetRighthandOffset(remainingVertices[i]) > NumericTolerance)
						{
							partitionedVertices.Add(remainingVertices[i]);
							remainingVertices.RemoveAt(i);
						}
					}

					if (partitionedVertices.Count > 0)
					{
						// Construct a new mesh from the remaining vertices, then merge into this mesh
						_Merge(Construct(partitionedVertices));
						anyVerticesAddedToMesh = true;
						break;
					}
				}

				if (!anyVerticesAddedToMesh)
				{
					_VerifyConvexHull();
					throw new Exception("Expected to find a vertex outside of the convex hull.");
				}
			}
		}
	}

	#endregion Insertion

	#region Removal

	/// <summary>
	/// Removes an edge and any triangles associated with that edge.
	/// </summary>
	/// <param name="a">A vertex endpoint of the edge.</param>
	/// <param name="b">A vertex endpoint of the edge.</param>
	private Edge<T, Vertex> _RemoveEdge(Vertex a, Vertex b)
	{
		Edge<T, Vertex>? edge = _Edges.FirstOrDefault(e => (e.Vertex1.Equals(a) && e.Vertex2.Equals(b)) || (e.Vertex2.Equals(a) && e.Vertex1.Equals(b)));
		if (edge == null)
			throw new Exception("Expected to find edge that does not exist.");
		_RemoveEdge(edge);
		return edge;
	}

	/// <summary>
	/// Removes an edge and any triangles associated with that edge.
	/// </summary>
	/// <param name="edge">The edge to remove.</param>
	private void _RemoveEdge(Edge<T, Vertex> edge)
	{
		_Edges.Remove(edge);

		if (edge.Left != null)
		{
			Triangle<T, Vertex> triangle = edge.Left;
			_Triangles.Remove(triangle);
			foreach (Edge<T, Vertex> otherEdge in triangle.Edges)
			{
				if (triangle.Equals(otherEdge.Left))
					otherEdge._Left = null;
				if (triangle.Equals(otherEdge.Right))
					otherEdge._Right = null;
			}
		}
		if (edge.Right != null)
		{
			Triangle<T, Vertex> triangle = edge.Right;
			_Triangles.Remove(triangle);
			foreach (Edge<T, Vertex> otherEdge in triangle.Edges)
			{
				if (triangle.Equals(otherEdge.Left))
					otherEdge._Left = null;
				if (triangle.Equals(otherEdge.Right))
					otherEdge._Right = null;
			}
		}
	}

	/// <summary>
	/// Rebuilds a Delaunay triangulation constrained by the edges of a polygon.
	/// </summary>
	/// <param name="edges">The edges of the polygon.</param>
	private void _RebuildConstrainedPolygon(IEnumerable<Edge<T, Vertex>> edges)
	{
		throw new NotImplementedException();
	}

	/// <summary>
	/// Removes vertices in a clique from the mesh.
	/// </summary>
	/// <param name="clique">A clique of vertices.</param>
	private void _RemoveClique(HashSet<Vertex> clique)
	{
		// Locate all edges and triangles connected to the vertices in the clique
		List<Edge<T, Vertex>> removedEdges = new List<Edge<T, Vertex>>(_Edges.Where(e => clique.Contains(e.Vertex1) || clique.Contains(e.Vertex2)));
		List<Triangle<T, Vertex>> removedTriangles = new List<Triangle<T, Vertex>>(removedEdges.SelectMany(e =>
		{
			List<Triangle<T, Vertex>> result = new List<Triangle<T, Vertex>>();
			if (e.Left != null)
				result.Add(e.Left);
			if (e.Right != null)
				result.Add(e.Right);
			return result;
		}));

		// Remove the vertices in the clique, as well as the edges and triangles attached to those vertices
		_Vertices.ExceptWith(clique);
		foreach (Edge<T, Vertex> removedEdge in removedEdges)
			_RemoveEdge(removedEdge);

		// Construct a polygon from the surrounding outer edges
		IEnumerable<Edge<T, Vertex>> outerEdges = _Edges.Where(e => removedTriangles.Any(removedTriangle => removedTriangle.Edges.Contains(e)));
		if (outerEdges.Any())
		{
			Dictionary<Vertex, Edge<T, Vertex>> outerEdgeFirstVertex = new Dictionary<Vertex, Edge<T, Vertex>>(outerEdges.Select(e => new KeyValuePair<Vertex, Edge<T, Vertex>>(e.Vertex1, e)));
			Dictionary<Vertex, Edge<T, Vertex>> outerEdgeLastVertex = new Dictionary<Vertex, Edge<T, Vertex>>(outerEdges.Select(e => new KeyValuePair<Vertex, Edge<T, Vertex>>(e.Vertex2, e)));
			while (outerEdgeFirstVertex.Count > 0)
			{
				Vertex firstVertex = outerEdgeFirstVertex.Keys.First();
				Vertex lastVertex = firstVertex;
				List<Vertex> polygon = new List<Vertex>();

				while (true)
				{
					Edge<T, Vertex>? precedingPolygonEdge, nextPolygonEdge;
					if (outerEdgeFirstVertex.TryGetValue(lastVertex, out nextPolygonEdge) && nextPolygonEdge != null)
					{
						outerEdgeFirstVertex.Remove(nextPolygonEdge.Vertex1);
						outerEdgeLastVertex.Remove(nextPolygonEdge.Vertex2);
						polygon.Add(nextPolygonEdge.Vertex2);
						lastVertex = nextPolygonEdge.Vertex2;
					}
					else if (outerEdgeLastVertex.TryGetValue(lastVertex, out nextPolygonEdge) && nextPolygonEdge != null)
					{
						outerEdgeFirstVertex.Remove(nextPolygonEdge.Vertex1);
						outerEdgeLastVertex.Remove(nextPolygonEdge.Vertex2);
						polygon.Add(nextPolygonEdge.Vertex1);
						lastVertex = nextPolygonEdge.Vertex1;
					}
					else if (outerEdgeLastVertex.TryGetValue(firstVertex, out precedingPolygonEdge) && precedingPolygonEdge != null)
					{
						outerEdgeFirstVertex.Remove(precedingPolygonEdge.Vertex1);
						outerEdgeLastVertex.Remove(precedingPolygonEdge.Vertex2);
						polygon.Insert(0, precedingPolygonEdge.Vertex1);
						firstVertex = precedingPolygonEdge.Vertex1;
					}
					else if (outerEdgeFirstVertex.TryGetValue(firstVertex, out precedingPolygonEdge) && precedingPolygonEdge != null)
					{
						outerEdgeFirstVertex.Remove(precedingPolygonEdge.Vertex1);
						outerEdgeLastVertex.Remove(precedingPolygonEdge.Vertex2);
						polygon.Insert(0, precedingPolygonEdge.Vertex2);
						firstVertex = precedingPolygonEdge.Vertex2;
					}
					else break;
				}

				if (firstVertex.Equals(lastVertex))
				{
					// Rebuild the polygon
					Mesh<T, Vertex> polygonMesh = SimplePolygonMesh<T, Vertex>.Construct(polygon, NumericTolerance);

					// Shove the edges and triangles into this mesh
					_Edges.UnionWith(polygonMesh.Edges);
					_Triangles.UnionWith(polygonMesh.Triangles);
				}
				else
				{
					// A chunk was taken out of the convex hull
					throw new NotImplementedException();
				}
			}
		}
	}

	public override void Remove(Vertex vertex)
	{
		if (_Vertices.Count <= 1)
		{
			_Vertices.Remove(vertex);
			return;
		}

		// A single vertex is a clique
		_RemoveClique(new HashSet<Vertex> { vertex });
	}

	public override void RemoveRange(IEnumerable<Vertex> vertices)
	{
		HashSet<Vertex> remainingVertices = new HashSet<Vertex>(vertices);
		while (remainingVertices.Count > 0)
		{
			HashSet<Vertex> convexHullVertices = new HashSet<Vertex>(_ConvexHull.Select(e => e.Vertex1));

			// Generate an arbitrary maximal clique
			HashSet<Vertex> clique = new HashSet<Vertex>();
			bool cliqueOnConvexHull = true;
			foreach (Vertex vertex in remainingVertices)
			{
				bool vertexOnConvexHull = convexHullVertices.Contains(vertex);
				if (cliqueOnConvexHull && !vertexOnConvexHull)
				{
					// Prioritize a clique not on the convex hull
					remainingVertices.UnionWith(clique);
					clique = new HashSet<Vertex> { vertex };
					remainingVertices.Remove(vertex);
					cliqueOnConvexHull = false;
				}
				else if (clique.Count == 0 || clique.All(v => FindExistingEdge(v, vertex) != null))
				{
					if (!cliqueOnConvexHull && vertexOnConvexHull)
						continue; // If the clique is not already on the convex hull, avoid adding a vertex from the convex hull

					clique.Add(vertex);
					remainingVertices.Remove(vertex);

					if (clique.Count == 3)
						break; // A clique in a triangulation can have size no greater than 3, so you can save some time by breaking the loop early
				}
			}

			// Remove that clique from the graph
			_RemoveClique(clique);
		}
	}

	#endregion Removal

}
