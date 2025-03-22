# DelaunayTriangulation
This is a library for performing Delaunay triangulations. The main motivations of this library that are not available (from what I could tell) in other C# libraries are as follows:
1. Provide support for different levels of numeric precision.
2. Provide the ability to store additional data about vertices in a way that is easy to retrieve.
3. Provide support for easily adding and removing vertices from a triangulation after initial construction.

## Documentation
### Mesh
A `Mesh` object is the Delaunay triangulation of a set of vertices. All `Mesh` types require two type arguments: `T` and `Vertex`. `T` is the floating-point type to use for numeric calculations (e.g. `float`, `double`). `Vertex` must be a type that implements the interface `IVertex2<T>`. One of the motivations behind the development of this library was to have support for different levels of precision, as well as to be able to store data other than coordinates associated with a vertex.

There are two kinds of `Mesh` objects: `ConvexHullMesh` (for basic Delaunay triangulations) and `SimplePolygonMesh` (for the Delaunay triangulation of a simple polygon). To construct a Mesh object, use one of the following static methods:

    ConvexHullMesh<T, Vertex>.Construct(IEnumerable<Vertex> vertices)
    SimplePolygonMesh<T, Vertex>.Construct(IEnumerable<Vertex> vertices)

#### Properties
    IEnumerable<Vertex> Vertices

Enumerates all vertices of the mesh.

    IEnumerable<Edge<T, Vertex>> Edges

Enumerates all edges of the mesh.

    IEnumerable<Triangle<T, Vertex>> Triangles

Enumerates all triangles of the mesh.

### ConvexHullMesh
`ConvexHullMesh` objects represent a basic Delaunay triangulation with no constraints. It currently uses the Guibas and Stolfi divide-and-conquer algorithm for initial construction, which is known to not be state-of-the-art in terms of efficiency but was relatively simple to implement. One of the motivations behind this library was to include support for adding and removing vertices from the mesh without needing to recalculate the entire triangulation.

#### Methods
    void Add(Vertex vertex)
    void AddRange(IEnumerable<Vertex> vertices)

Inserts vertices into the mesh and recalculates the triangulation.

    void Remove(Vertex vertex)
    void RemoveRange(IEnumerable<Vertex> vertices)

Removes vertices from the mesh and recalculates the triangulation.

### SimplePolygonMesh
`SimplePolygonMesh` objects represent a Delaunay triangulation constrained within the bounds of a simple polygon.

### Edge
The `Edge` class represents an edge in the Delaunay triangulation. It requires the same type arguments as `Mesh`.

#### Properties
    IEnumerable<Vertex> Vertices

The vertex endpoints of the edge.

    Triangle<T, Vertex>? Left

The triangle on the lefthand-side of the edge (when oriented so that the first vertex in `Vertices` is on the bottom and the second vertex is on the top).

    Triangle<T, Vertex>? Right

The triangle on the righthand-side of the edge (when oriented so that the first vertex in `Vertices` is on the bottom and the second vertex is on the top).

#### Methods
    T GetAngularDifference(Edge<T, Vertex> other)

Calculates the counter-clockwise angle of rotation from this edge to the given edge. (The given edge must have at least one vertex in common with this edge.)

### Triangle
The Triangle class represents a triangle in the Delaunay triangulation.

#### Properties
    IEnumerable<Vertex> Vertices

The vertices of the triangle.

    IEnumerable<Triangle<T, Vertex>> AdjacentTriangles

The triangles adjacent to this one.

    IVertex2<T> CircumcircleCenter

The center of the circumcircle defined by the three vertices of the triangle.

    T Area

The area of this triangle.
