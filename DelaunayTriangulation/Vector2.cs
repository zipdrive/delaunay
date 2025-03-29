using System;
using System.Numerics;
using System.Security.Cryptography.X509Certificates;

namespace DelaunayTriangulation;

/// <summary>
/// A 2D vector.
/// </summary>
/// <typeparam name="T">The floating-point type used for numeric calculations.</typeparam>
public class Vector2<T> where T : IFloatingPointIeee754<T>
{
    /// <summary>
    /// The x component of the vector.
    /// </summary>
    public required T X;

    /// <summary>
    /// The y component of the vector.
    /// </summary>
    public required T Y;

    /// <summary>
    /// Calculates the length of the vector, squared.
    /// </summary>
    /// <returns></returns>
    public T LengthSquared => X * X + Y * Y;

    /// <summary>
    /// Calculates the length of the vector.
    /// </summary>
    public T Length => T.Sqrt(LengthSquared);

	/// <summary>
	/// Calculates the vector from the given start position to the given end position.
	/// </summary>
	/// <typeparam name="Vertex">A vertex type.</typeparam>
	/// <param name="start">The start of the vector.</param>
	/// <param name="end">The end of the vector.</param>
	/// <returns>The vector from the given start position to the given end position.</returns>
	public static Vector2<T> VectorDifference(IVertex2<T> start, IVertex2<T> end) => new Vector2<T>
	{
		X = end.X - start.X,
		Y = end.Y - start.Y
	};

    /// <summary>
    /// Adds the components of the vectors and returns the result.
    /// </summary>
    /// <param name="lhs">The vector on the left-hand side.</param>
    /// <param name="rhs">The vector on the right-hand side.</param>
    /// <returns>The vector result of component-wise addition.</returns>
    public static Vector2<T> operator+(Vector2<T> lhs, Vector2<T> rhs) => new Vector2<T>
    {
        X = lhs.X + rhs.X,
        Y = lhs.Y + rhs.Y
    };

    /// <summary>
    /// Subtracts the components of the vectors and returns the result.
    /// </summary>
    /// <param name="lhs">The vector on the left-hand side.</param>
    /// <param name="rhs">The vector on the right-hand side.</param>
    /// <returns>The vector result of component-wise subtraction.</returns>
    public static Vector2<T> operator-(Vector2<T> lhs, Vector2<T> rhs) => new Vector2<T>
    {
        X = lhs.X - rhs.X,
        Y = lhs.Y - rhs.Y
    };

    /// <summary>
    /// Multiplies the components of the vectors and returns the result.
    /// </summary>
    /// <param name="lhs">The vector on the left-hand side.</param>
    /// <param name="rhs">The vector on the right-hand side.</param>
    /// <returns>The vector result of component-wise multiplication.</returns>
    public static Vector2<T> operator*(Vector2<T> lhs, Vector2<T> rhs) => new Vector2<T>
    {
        X = lhs.X * rhs.X,
        Y = lhs.Y * rhs.Y
    };

    /// <summary>
    /// Multiplies the components of the vector by a scalar and returns the result.
    /// </summary>
    /// <param name="lhs">The vector on the left-hand side.</param>
    /// <param name="rhs">The scalar on the right-hand side.</param>
    /// <returns>The vector result of scalar multiplication.</returns>
    public static Vector2<T> operator*(Vector2<T> lhs, T rhs) => new Vector2<T>
    {
        X = lhs.X * rhs,
        Y = lhs.Y * rhs
    };

    /// <summary>
    /// Multiplies the components of the vector by a scalar and returns the result.
    /// </summary>
    /// <param name="lhs">The scalar on the left-hand side.</param>
    /// <param name="rhs">The vector on the right-hand side.</param>
    /// <returns>The vector result of scalar multiplication.</returns>
    public static Vector2<T> operator*(T lhs, Vector2<T> rhs) => rhs * lhs;

    /// <summary>
    /// Divides the components of the vectors and returns the result.
    /// </summary>
    /// <param name="lhs">The vector on the left-hand side.</param>
    /// <param name="rhs">The vector on the right-hand side.</param>
    /// <returns>The vector result of component-wise division.</returns>
    public static Vector2<T> operator/(Vector2<T> lhs, Vector2<T> rhs) => new Vector2<T>
    {
        X = lhs.X / rhs.X,
        Y = lhs.Y / rhs.Y
    };

    /// <summary>
    /// Divides the components of the vector by a scalar and returns the result.
    /// </summary>
    /// <param name="lhs">The vector on the left-hand side.</param>
    /// <param name="rhs">The scalar on the right-hand side.</param>
    /// <returns>The vector result of scalar division.</returns>
    public static Vector2<T> operator/(Vector2<T> lhs, T rhs) => new Vector2<T>
    {
        X = lhs.X / rhs,
        Y = lhs.Y / rhs
    };

	/// <summary>
	/// Inverts the sign of the vector and returns the result.
	/// </summary>
	/// <param name="vector">The vector to invert the sign of.</param>
	/// <returns>The vector result of sign inversion.</returns>
	public static Vector2<T> operator -(Vector2<T> vector) => new Vector2<T>
	{
		X = -vector.X,
		Y = -vector.Y
	};

    /// <summary>
    /// Calculates the dot product with another vector.
    /// </summary>
    /// <param name="other">The other vector.</param>
    /// <returns>The dot product of the two vectors.</returns>
    public T Dot(Vector2<T> other) => X * other.X + Y * other.Y;

	/// <summary>
	/// Calculates the magnitude of the cross product with another vector.
	/// </summary>
	/// <param name="other">The other vector.</param>
	/// <returns>The magnitude of the cross product of the two vectors.</returns>
	public T CrossMagnitude(Vector2<T> other) => X * other.Y - Y * other.X;

    /// <summary>
    /// Normalizes the vector.
    /// </summary>
    /// <returns>The normalized vector.</returns>
    public Vector2<T> Normalize() => this / Length;

    /// <summary>
    /// Calculates the counter-clockwise angle between two vectors.
    /// </summary>
    /// <param name="other">The other vector.</param>
    /// <returns>The counter-clockwise angle between this vector and the other vector.</returns>
    public T Angle(Vector2<T> other)
    {
        T angle = T.Acos(Dot(other) / (Length * other.Length));
        T sgn = X * other.Y - Y * other.X;
        return sgn < T.Zero ? T.Tau - angle : angle;
    }
}
