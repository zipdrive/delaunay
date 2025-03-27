using OxyPlot;
using OxyPlot.Series;
using OxyPlot.WindowsForms;
using DelaunayTriangulation;
using OxyPlot.Annotations;
using OxyPlot.Axes;

namespace DelaunayTriangulationTestCase1
{
	public partial class Form1 : Form
	{
		PlotView plot1;

		public Form1()
		{
#if TESTCASE_MERGE_ALGORITHM

			#region Setup
			this.plot1 = new OxyPlot.WindowsForms.PlotView();
			this.SuspendLayout();
			// 
			// plot1
			// 
			this.plot1.Dock = System.Windows.Forms.DockStyle.Bottom;
			this.plot1.Location = new System.Drawing.Point(0, 0);
			this.plot1.Name = "plot1";
			this.plot1.PanCursor = System.Windows.Forms.Cursors.Hand;
			this.plot1.Size = new System.Drawing.Size(500, 500);
			this.plot1.TabIndex = 0;
			this.plot1.Text = "plot1";
			this.plot1.ZoomHorizontalCursor = System.Windows.Forms.Cursors.SizeWE;
			this.plot1.ZoomRectangleCursor = System.Windows.Forms.Cursors.SizeNWSE;
			this.plot1.ZoomVerticalCursor = System.Windows.Forms.Cursors.SizeNS;
			this.Controls.Add(this.plot1);

			InitializeComponent();
			var model = new PlotModel { Title = "Test" };
			this.plot1.Model = model;
			#endregion Setup

			// Randomly initialize some vertices
			const int NUM_VERTICES = 100;
			Random rand = new Random();
			List<Vertex2> vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_VERTICES; ++k)
				vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});

			// Construct a mesh from them
			ConvexHullMesh = ConvexHullMesh<double, Vertex2>.Construct(vertices);

			#region Display
			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			// Draw edges
			foreach (Edge<double, Vertex2> edge in ConvexHullMesh.Edges)
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in ConvexHullMesh.Triangles)
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0xFF, 0x00, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0xFF, 0x00, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
			
			// Draw vertices
			foreach (Vertex2 vertex in ConvexHullMesh.Vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}
			#endregion Display

#endif

#if TESTCASE_INSERT_MANY_ALGORITHM

			#region Setup
			this.plot1 = new OxyPlot.WindowsForms.PlotView();
			this.SuspendLayout();
			// 
			// plot1
			// 
			this.plot1.Dock = System.Windows.Forms.DockStyle.Bottom;
			this.plot1.Location = new System.Drawing.Point(0, 0);
			this.plot1.Name = "plot1";
			this.plot1.PanCursor = System.Windows.Forms.Cursors.Hand;
			this.plot1.Size = new System.Drawing.Size(500, 500);
			this.plot1.TabIndex = 0;
			this.plot1.Text = "plot1";
			this.plot1.ZoomHorizontalCursor = System.Windows.Forms.Cursors.SizeWE;
			this.plot1.ZoomRectangleCursor = System.Windows.Forms.Cursors.SizeNWSE;
			this.plot1.ZoomVerticalCursor = System.Windows.Forms.Cursors.SizeNS;
			this.Controls.Add(this.plot1);

			InitializeComponent();
			var model = new PlotModel { Title = "Test" };
			this.plot1.Model = model;
			#endregion Setup

			// Randomly initialize some vertices
			const int NUM_INITIAL_VERTICES = 20;
			Random rand = new Random();
			List<Vertex2> vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_INITIAL_VERTICES; ++k)
				vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});


			// Construct a mesh from them
			ConvexHullMesh = ConvexHullMesh<double, Vertex2>.Construct(vertices);
			var initialEdges = new HashSet<Edge<double, Vertex2>>(ConvexHullMesh.Edges);
			var initialTriangles = new HashSet<Triangle<double, Vertex2>>(ConvexHullMesh.Triangles);

			// Add new vertices
			const int NUM_ADDITIONAL_VERTICES = 6;
			List<Vertex2> newVertices = new List<Vertex2>();
			for (int k = 0; k < NUM_ADDITIONAL_VERTICES; ++k)
			{
				newVertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});
			}
			ConvexHullMesh.AddRange(newVertices.Take(NUM_ADDITIONAL_VERTICES / 2));
			ConvexHullMesh.AddRange(newVertices.Skip(NUM_ADDITIONAL_VERTICES / 2));

			var finalEdges = new HashSet<Edge<double, Vertex2>>(ConvexHullMesh.Edges);
			var finalTriangles = new HashSet<Triangle<double, Vertex2>>(ConvexHullMesh.Triangles);

			#region Display
			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			// Draw edges
			foreach (Edge<double, Vertex2> edge in initialEdges.Intersect(finalEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0x00, 0x80, 0xFF),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
			foreach (Edge<double, Vertex2> edge in finalEdges.Except(initialEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0xFF, 0x80, 0x00),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in initialTriangles.Intersect(finalTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0x00, 0xFF),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0x00, 0xFF)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
			foreach (Triangle<double, Vertex2> triangle in finalTriangles.Except(initialTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0xFF, 0x00, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0xFF, 0x00, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}

			// Draw vertices
			foreach (Vertex2 vertex in vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}
			foreach (Vertex2 vertex in newVertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x80, 0x80)
				});
			}
			#endregion Display

#endif

#if TESTCASE_REMOVE_MANY_ALGORITHM

			#region Setup
			this.plot1 = new OxyPlot.WindowsForms.PlotView();
			this.SuspendLayout();
			// 
			// plot1
			// 
			this.plot1.Dock = System.Windows.Forms.DockStyle.Bottom;
			this.plot1.Location = new System.Drawing.Point(0, 0);
			this.plot1.Name = "plot1";
			this.plot1.PanCursor = System.Windows.Forms.Cursors.Hand;
			this.plot1.Size = new System.Drawing.Size(500, 500);
			this.plot1.TabIndex = 0;
			this.plot1.Text = "plot1";
			this.plot1.ZoomHorizontalCursor = System.Windows.Forms.Cursors.SizeWE;
			this.plot1.ZoomRectangleCursor = System.Windows.Forms.Cursors.SizeNWSE;
			this.plot1.ZoomVerticalCursor = System.Windows.Forms.Cursors.SizeNS;
			this.Controls.Add(this.plot1);

			InitializeComponent();
			var model = new PlotModel { Title = "Test" };
			this.plot1.Model = model;
			#endregion Setup

			// Randomly initialize some vertices
			const int NUM_INITIAL_VERTICES = 10;
			Random rand = new Random();
			List<Vertex2> vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_INITIAL_VERTICES; ++k)
				vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});


			// Construct a mesh from them
			ConvexHullMesh<double, Vertex2> mesh = ConvexHullMesh<double, Vertex2>.Construct(vertices);
			var initialEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var initialTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			// Add new vertices
			const int NUM_REMOVED_VERTICES = 1;
			List<Vertex2> removeVertices = new List<Vertex2>(vertices.Take(NUM_REMOVED_VERTICES));
			mesh.RemoveRange(removeVertices.Take(NUM_REMOVED_VERTICES / 2));
			mesh.RemoveRange(removeVertices.Skip(NUM_REMOVED_VERTICES / 2));

			var finalEdges = new HashSet<Edge<double, Vertex2>>(mesh.Edges);
			var finalTriangles = new HashSet<Triangle<double, Vertex2>>(mesh.Triangles);

			#region Display
			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			// Draw edges
			foreach (Edge<double, Vertex2> edge in initialEdges.Intersect(finalEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0x00, 0x80, 0xFF),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
			foreach (Edge<double, Vertex2> edge in finalEdges.Except(initialEdges))
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					Color = OxyColor.FromRgb(0xFF, 0x80, 0x00),
					StrokeThickness = 3.0,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}
            foreach (Edge<double, Vertex2> edge in mesh.Boundary)
            {
                LineAnnotation edgeLine = new LineAnnotation
                {
                    Type = LineAnnotationType.LinearEquation,
                    LineStyle = LineStyle.Solid,
                    Color = OxyColor.FromArgb(0xFF, 0x00, 0x00, 0x00),
                    StrokeThickness = 3.0,
                    MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
                    MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
                    MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
                    MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
                };
                if (edgeLine.MinimumX == edge.Vertex1.X)
                {
                    edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
                    edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
                }
                else
                {
                    edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
                    edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
                }
                model.Annotations.Add(edgeLine);
            }

            // Draw triangles
            foreach (Triangle<double, Vertex2> triangle in initialTriangles.Intersect(finalTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0x00, 0xFF),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0x00, 0xFF)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
			foreach (Triangle<double, Vertex2> triangle in finalTriangles.Except(initialTriangles))
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0x00, 0xFF, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0x00, 0xFF, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
            foreach (Triangle<double, Vertex2> triangle in initialTriangles.Except(finalTriangles))
            {
                PolygonAnnotation triangleLines = new PolygonAnnotation
                {
                    LineStyle = LineStyle.Dot,
                    Stroke = OxyColor.FromRgb(0xFF, 0x00, 0x00),
                    StrokeThickness = 2.0,
                    Fill = OxyColor.FromArgb(0x10, 0xFF, 0x00, 0x00)
                };
                triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
                model.Annotations.Add(triangleLines);
            }

            // Draw vertices
            foreach (Vertex2 vertex in mesh.Vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}

            foreach (Vertex2 vertex in removeVertices)
            {
                model.Annotations.Add(new PointAnnotation
                {
                    X = vertex.X,
                    Y = vertex.Y,
                    Size = 3.0,
                    Fill = OxyColor.FromRgb(0xFF, 0x00, 0x00)
                });
            }
            #endregion Display

#endif

        }
	}
}
