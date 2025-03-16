using System;
using System.Numerics;
using System.Security.Cryptography.X509Certificates;

namespace DelaunayTriangulation;

internal class Vector2<T> where T : IFloatingPointIeee754<T>
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
    /// Calculates the dot product with another vector.
    /// </summary>
    /// <param name="other">The other vector.</param>
    /// <returns>The dot product of the two vectors.</returns>
    public T Dot(Vector2<T> other) => X * other.X + Y * other.Y;

    /// <summary>
    /// Normalizes the vector.
    /// </summary>
    /// <returns>The normalized vector.</returns>
    public Vector2<T> Normalize() => this / Length;
}
