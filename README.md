# DelaunayTriangulation
This is a library for performing Delaunay triangulations. The main motivations of this library that are not available (from what I could tell) in other C# libraries are as follows:
1. Provide support for different levels of numeric precision.
2. Provide the ability to store additional data about vertices in a way that is easy to retrieve.
3. Provide support for easily adding and removing vertices from a triangulation after initial construction.

The primary interface for interacting with this library is the `ConvexHullMesh` class, which represents a Delaunay triangulation of vertices.

## Documentation
### Mesh
A `Mesh` object is the Delaunay triangulation of a set of vertices. All `Mesh` types require two type arguments: `T` and `Vertex`. `T` is the floating-point type to use for numeric calculations (e.g. `float`, `double`). `Vertex` must be a type that implements the interface `IPoint2<T>`. One of the motivations behind the development of this library was to have support for different levels of precision, as well as to be able to store data other than coordinates associated with a vertex.

`Mesh` is an abstract class, of which the only current implementation is `ConvexHullMesh` (for basic Delaunay triangulations).

#### Properties
    IEnumerable<Vertex> Vertices { get; }

Enumerates all vertices of the mesh.

    IEnumerable<Edge<T, Vertex>> Edges { get; }

Enumerates all edges of the mesh.

    IEnumerable<Triangle<T, Vertex>> Triangles { get; }

Enumerates all triangles of the mesh.

### ConvexHullMesh
`ConvexHullMesh` objects represent a basic Delaunay triangulation with no constraints. It currently uses the Guibas and Stolfi divide-and-conquer algorithm for initial construction, which is known to not be state-of-the-art in terms of efficiency but was relatively simple to implement. One of the motivations behind this library was to include support for adding and removing vertices from the mesh without needing to recalculate the entire triangulation.

#### Properties
    IEnumerable<Vertex> Vertices { get; }

Enumerates all vertices of the mesh.

    IEnumerable<Edge<T, Vertex>> Edges { get; }

Enumerates all edges of the mesh.

    IEnumerable<Triangle<T, Vertex>> Triangles { get; }

Enumerates all triangles of the mesh.

#### Static Methods
    ConvexHullMesh<T, Vertex> Construct(IEnumerable<Vertex> vertices, T numericTolerance = 4 * T.Epsilon);

Constructs the Delaunay triangulation of the given vertices. Uses the Guibas and Stolfi divide-and-conquer algorithm.
* vertices: The vertices to construct a Delaunay triangulation for.
* numericTolerance: Controls the range of values that are considered "zero". It defaults to 4 times the smallest non-zero number that can be represented by type T.

#### Methods
    void Add(Vertex vertex);
    void AddRange(IEnumerable<Vertex> vertices);

Inserts vertices into the mesh and recalculates the triangulation. Uses the Guibas and Stolfi divide-and-conquer algorithm (https://doi.org/10.1145/282918.282923) for vertices outside of the convex hull, and incremental insertion for vertices inside of the convex hull.

    void Remove(Vertex vertex);
    void RemoveRange(IEnumerable<Vertex> vertices);

Removes vertices from the mesh and recalculates the triangulation. Uses the ear-queue algorithm (https://doi.org/10.1145/304893.304969).

### IPoint2
The `IPoint2` interface represents a 2D point. It accepts a single type argument, `T`, which is the floating-point type to use for numeric calculations (e.g. `float`, `double`).

A basic implementation of the interface, `SimplePoint2`, is available in the library.

#### Properties
    T X { get; }

The x-coordinate of the point.

    T Y { get; }

The y-coordinate of the point.

### Edge
The `Edge` class represents an edge in the Delaunay triangulation. It requires the same type arguments as `Mesh` (i.e. `T` and `Vertex`).

#### Properties
    IEnumerable<Vertex> Vertices { get; }

The vertex endpoints of the edge.

    Triangle<T, Vertex>? Left { get; }

The triangle on the lefthand-side of the edge (when oriented so that the first vertex in `Vertices` is on the bottom and the second vertex is on the top).

    Triangle<T, Vertex>? Right { get; }

The triangle on the righthand-side of the edge (when oriented so that the first vertex in `Vertices` is on the bottom and the second vertex is on the top).

#### Methods
    T GetAngularDifference(Edge<T, Vertex> other);

Calculates the counter-clockwise angle of rotation from this edge to the given edge. (The given edge must have at least one vertex in common with this edge.)

### Triangle
The Triangle class represents a triangle in the Delaunay triangulation. It requires the same type arguments as `Mesh` (i.e. `T` and `Vertex`).

#### Properties
    IEnumerable<Vertex> Vertices { get; }

The vertices of the triangle.

    IEnumerable<Edge<T, Vertex>> Edges { get; }

The edges of the triangle.

    IEnumerable<Triangle<T, Vertex>> AdjacentTriangles { get; }

The triangles adjacent to this one.

    SimplePoint2<T> CircumcircleCenter { get; }

The center of the circumcircle defined by the three vertices of the triangle.

    T CircumcircleRadiusSquared { get; }

The squared value of the radius of the circumcircle defined by the three vertices of the triangle.

    T Area { get; }

The area of this triangle.
